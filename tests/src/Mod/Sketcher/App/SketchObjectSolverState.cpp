// SPDX-License-Identifier: LGPL-2.1-or-later

// Solver state (DoF, diagnosis, solved geometry) of a sketch after recompute, undo/redo and
// restore, including conflicting, redundant, external-geometry and expression-driven sketches.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>

#include <App/Application.h>
#include <App/Document.h>
#include <App/Expression.h>
#include <App/ObjectIdentifier.h>
#include <Base/Placement.h>
#include <Mod/Part/App/FeaturePartBox.h>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/TopoShape.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/GeoEnum.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include "SketcherTestHelpers.h"

using namespace Sketcher;

namespace
{

std::unique_ptr<Constraint> makeConstraint(
    ConstraintType type,
    int first,
    PointPos firstPos = PointPos::none,
    int second = GeoEnum::GeoUndef,
    PointPos secondPos = PointPos::none,
    double value = 0.0
)
{
    auto constraint = std::make_unique<Constraint>();
    constraint->Type = type;
    constraint->First = first;
    constraint->FirstPos = firstPos;
    constraint->Second = second;
    constraint->SecondPos = secondPos;
    constraint->setValue(value);
    return constraint;
}

void addLine(SketchObject* sketch, const Base::Vector3d& start, const Base::Vector3d& end)
{
    Part::GeomLineSegment line;
    line.setPoints(start, end);
    sketch->addGeometry(&line);
}

// Axis-aligned rectangle with dimensioned width (its 9th constraint) and height, its lower left
// corner placed by DistanceY and optionally DistanceX from the origin.
void addRectangle(SketchObject* sketch, double x, double y, double w, double h, bool placeX = true)
{
    const int g = sketch->getHighestCurveIndex() + 1;
    const Base::Vector3d p[] = {
        Base::Vector3d(x, y, 0),
        Base::Vector3d(x + w, y, 0),
        Base::Vector3d(x + w, y + h, 0),
        Base::Vector3d(x, y + h, 0)
    };
    for (int i = 0; i < 4; ++i) {
        addLine(sketch, p[i], p[(i + 1) % 4]);
    }
    for (int i = 0; i < 4; ++i) {
        sketch->addConstraint(
            makeConstraint(Coincident, g + i, PointPos::end, g + (i + 1) % 4, PointPos::start)
        );
    }
    sketch->addConstraint(makeConstraint(Horizontal, g));
    sketch->addConstraint(makeConstraint(Horizontal, g + 2));
    sketch->addConstraint(makeConstraint(Vertical, g + 1));
    sketch->addConstraint(makeConstraint(Vertical, g + 3));
    sketch->addConstraint(makeConstraint(DistanceX, g, PointPos::start, g, PointPos::end, w));
    sketch->addConstraint(makeConstraint(DistanceY, g + 1, PointPos::start, g + 1, PointPos::end, h));
    if (placeX) {
        sketch->addConstraint(makeConstraint(DistanceX, -1, PointPos::start, g, PointPos::start, x));
    }
    sketch->addConstraint(makeConstraint(DistanceY, -1, PointPos::start, g, PointPos::start, y));
}

constexpr int widthOfFirst = 8;  // index of the first rectangle's width constraint

enum class Defect
{
    None,
    Conflicting,  // a second, different width of the first rectangle
    Redundant,    // a second Horizontal constraint on the first rectangle's bottom line
};

struct Diagnosis
{
    int dof;
    std::vector<int> conflicting;  // 1-based constraint indices, as the solver reports them
    std::vector<int> redundant;
};

// Two fully constrained rectangles, the first 10 mm wide with its right side at x = 11,
// optionally with one defective extra constraint (index 24). Returns the expected diagnosis.
Diagnosis buildSketch(SketchObject* sketch, Defect defect)
{
    addRectangle(sketch, 1, 1, 10, 5);
    addRectangle(sketch, 20, 1, 7, 6);
    if (defect == Defect::Conflicting) {
        sketch->addConstraint(makeConstraint(DistanceX, 0, PointPos::start, 0, PointPos::end, 12.0));
        return {-1, {widthOfFirst + 1, 25}, {}};
    }
    if (defect == Defect::Redundant) {
        sketch->addConstraint(makeConstraint(Horizontal, 0));
        return {0, {}, {25}};
    }
    return {0, {}, {}};
}

void expectHeldDiagnosis(const SketchObject* sketch, const Diagnosis& expected)
{
    EXPECT_EQ(sketch->getLastDoF(), expected.dof);
    EXPECT_EQ(sketch->getLastConflicting(), expected.conflicting);
    EXPECT_EQ(sketch->getLastRedundant(), expected.redundant);
}

// Checks the diagnosis held by the sketch, and that a fresh solve of the same geometry and
// constraints gives the same diagnosis and leaves every line where it is.
void expectDiagnosis(SketchObject* sketch, const Diagnosis& expected)
{
    expectHeldDiagnosis(sketch, expected);
    std::vector<Base::Vector3d> before;
    for (int geoId = 0; geoId <= sketch->getHighestCurveIndex(); ++geoId) {
        before.push_back(sketch->getPoint(geoId, PointPos::start));
        before.push_back(sketch->getPoint(geoId, PointPos::end));
    }

    sketch->solve();

    expectHeldDiagnosis(sketch, expected);
    for (int geoId = 0; geoId <= sketch->getHighestCurveIndex(); ++geoId) {
        EXPECT_LT((sketch->getPoint(geoId, PointPos::start) - before[2 * geoId]).Length(), 1e-9);
        EXPECT_LT((sketch->getPoint(geoId, PointPos::end) - before[2 * geoId + 1]).Length(), 1e-9);
    }
}

double rightOfFirst(const SketchObject* sketch)
{
    return sketch->getPoint(0, PointPos::end).x;
}

const Defect allDefects[] = {Defect::None, Defect::Conflicting, Defect::Redundant};

}  // namespace

TEST_F(SketchObjectTest, solverStateAfterRecompute)
{
    for (Defect defect : allDefects) {
        SCOPED_TRACE(static_cast<int>(defect));
        // Arrange
        getObject()->deleteAllGeometry();
        const Diagnosis expected = buildSketch(getObject(), defect);

        // Act
        getObject()->getDocument()->recompute();

        // Assert
        EXPECT_EQ(getObject()->isValid(), defect == Defect::None);
        EXPECT_NEAR(rightOfFirst(getObject()), 11.0, 1e-9);
        expectDiagnosis(getObject(), expected);
    }
}

TEST_F(SketchObjectTest, solverStateAfterUndoRedoOfDatumChange)
{
    // Arrange
    auto* doc = getObject()->getDocument();
    const Diagnosis expected = buildSketch(getObject(), Defect::None);
    doc->recompute();
    doc->openTransaction("Change datum");
    ASSERT_EQ(getObject()->setDatum(widthOfFirst, 12.0), SketchSolveStatus::Success);
    doc->commitTransaction();
    doc->recompute();

    for (bool undo : {true, false}) {
        SCOPED_TRACE(undo ? "undo" : "redo");
        // Act
        ASSERT_TRUE(undo ? doc->undo() : doc->redo());

        // Assert: the solver state is up to date before and after the recompute
        const double right = undo ? 11.0 : 13.0;
        EXPECT_NEAR(rightOfFirst(getObject()), right, 1e-9);
        expectHeldDiagnosis(getObject(), expected);
        doc->recompute();
        EXPECT_TRUE(getObject()->isValid());
        EXPECT_NEAR(rightOfFirst(getObject()), right, 1e-9);
        expectDiagnosis(getObject(), expected);
    }
}

TEST_F(SketchObjectTest, solverStateAfterUndoRedoOfAddedLine)
{
    for (Defect defect : allDefects) {
        SCOPED_TRACE(static_cast<int>(defect));
        // Arrange
        auto* doc = getObject()->getDocument();
        getObject()->deleteAllGeometry();
        const Diagnosis expected = buildSketch(getObject(), defect);
        // with the free line, the conflicting sketch reports 4 DoF as well (baseline behaviour)
        const Diagnosis withLine {4, expected.conflicting, expected.redundant};
        doc->recompute();
        doc->openTransaction("Add line");
        addLine(getObject(), Base::Vector3d(-50, -50, 0), Base::Vector3d(-40, -45, 0));
        doc->commitTransaction();
        doc->recompute();

        for (bool undo : {true, false}) {
            SCOPED_TRACE(undo ? "undo" : "redo");
            // Act
            ASSERT_TRUE(undo ? doc->undo() : doc->redo());

            // Assert
            EXPECT_EQ(getObject()->getHighestCurveIndex(), undo ? 7 : 8);
            expectHeldDiagnosis(getObject(), undo ? expected : withLine);
            doc->recompute();
            EXPECT_EQ(getObject()->isValid(), defect == Defect::None);
            expectDiagnosis(getObject(), undo ? expected : withLine);
        }
    }
}

TEST_F(SketchObjectTest, solverStateAfterRestore)
{
    auto& app = App::GetApplication();
    for (Defect defect : allDefects) {
        SCOPED_TRACE(static_cast<int>(defect));
        // Arrange
        const std::string sourceName = app.getUniqueDocumentName("SolverStateSource");
        auto* source = app.newDocument(sourceName.c_str(), "testUser");
        const Diagnosis expected = buildSketch(source->addObject<SketchObject>("Sketch"), defect);
        source->recompute();
        ASSERT_TRUE(source->saveAs(App::Application::getTempFileName().c_str()));
        const std::string savedPath = source->getFileName();
        app.closeDocument(sourceName.c_str());

        // Act
        auto* restored = app.openDocument(savedPath.c_str());
        ASSERT_NE(restored, nullptr);
        auto* sketch = freecad_cast<SketchObject*>(restored->getObject("Sketch"));
        ASSERT_NE(sketch, nullptr);
        sketch->touch();
        restored->recompute();

        // Assert
        EXPECT_EQ(sketch->isValid(), defect == Defect::None);
        EXPECT_NEAR(rightOfFirst(sketch), 11.0, 1e-9);
        expectDiagnosis(sketch, expected);
        app.closeDocument(restored->getName());
    }
}

TEST_F(SketchObjectTest, recomputeRevalidatesConstraintIndices)
{
    // Arrange: an unmanaged assignment referencing a geometry that does not exist yet hides the
    // constraints; a recompute after adding that geometry must take them into account.
    auto* doc = getObject()->getDocument();
    addLine(getObject(), Base::Vector3d(0, 0, 0), Base::Vector3d(5, 1, 0));
    doc->recompute();
    // the rvalue overload takes ownership of the constraints
    getObject()->Constraints.setValues(
        std::vector<Constraint*> {
            makeConstraint(Horizontal, 0).release(),
            makeConstraint(Horizontal, 1).release()
        }
    );
    ASSERT_TRUE(getObject()->Constraints.getValues().empty());
    addLine(getObject(), Base::Vector3d(0, 3, 0), Base::Vector3d(5, 4, 0));

    // Act
    doc->recompute();

    // Assert
    EXPECT_EQ(getObject()->Constraints.getValues().size(), 2U);
    EXPECT_TRUE(getObject()->isValid());
    EXPECT_NEAR(getObject()->getPoint(1, PointPos::end).y, 3.0, 1e-9);
    expectDiagnosis(getObject(), {6, {}, {}});
}

TEST_F(SketchObjectTest, recomputeFollowsExternalGeometry)
{
    // Arrange: a rectangle placed 25 mm right of an external vertical line
    auto* doc = getObject()->getDocument();
    addRectangle(getObject(), 20, 1, 7, 6, /*placeX*/ false);
    auto* external = doc->addObject<Part::Feature>("External");
    external->Shape.setValue(
        Part::TopoShape(BRepBuilderAPI_MakeEdge(gp_Pnt(-5, -5, 0), gp_Pnt(-5, 15, 0)).Edge())
    );
    doc->recompute();
    ASSERT_EQ(getObject()->addExternal(external, "Edge1"), 0);
    getObject()->addConstraint(
        makeConstraint(DistanceX, GeoEnum::RefExt, PointPos::start, 0, PointPos::start, 25.0)
    );
    doc->recompute();
    ASSERT_NEAR(getObject()->getPoint(0, PointPos::start).x, 20.0, 1e-9);

    // Act
    external->Placement.setValue(Base::Placement(Base::Vector3d(-2, 0, 0), Base::Rotation()));
    doc->recompute();

    // Assert
    EXPECT_TRUE(getObject()->isValid());
    EXPECT_NEAR(getObject()->getPoint(GeoEnum::RefExt, PointPos::start).x, -7.0, 1e-9);
    EXPECT_NEAR(getObject()->getPoint(0, PointPos::start).x, 18.0, 1e-9);
    expectDiagnosis(getObject(), {0, {}, {}});
}

TEST_F(SketchObjectTest, recomputeFollowsExpressionDrivenConstraint)
{
    // Arrange: the first rectangle's width is bound to the length of a box
    auto* doc = getObject()->getDocument();
    buildSketch(getObject(), Defect::None);
    auto* box = doc->addObject<Part::Box>("Box");
    box->Length.setValue(10.0);
    getObject()->setExpression(
        getObject()->Constraints.createPath(widthOfFirst),
        App::Expression::parse(getObject(), "Box.Length")
    );
    doc->recompute();
    ASSERT_NEAR(rightOfFirst(getObject()), 11.0, 1e-9);

    // Act
    box->Length.setValue(12.0);
    doc->recompute();

    // Assert
    EXPECT_TRUE(getObject()->isValid());
    EXPECT_DOUBLE_EQ(getObject()->Constraints.getValues()[widthOfFirst]->getValue(), 12.0);
    EXPECT_NEAR(rightOfFirst(getObject()), 13.0, 1e-9);
    expectDiagnosis(getObject(), {0, {}, {}});
}
