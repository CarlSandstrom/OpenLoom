#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"

#include "Export/VtkGrid.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace Meshing;

namespace
{

class ScopedEnvironmentOverride
{
public:
    ScopedEnvironmentOverride(const std::string& name, const std::string& value) :
        name_(name)
    {
        auto previous = std::getenv(name_.c_str());
        hadPreviousValue_ = previous != nullptr;
        if (hadPreviousValue_) previousValue_ = previous;
        setenv(name_.c_str(), value.c_str(), 1);
    }

    ~ScopedEnvironmentOverride()
    {
        if (hadPreviousValue_)
            setenv(name_.c_str(), previousValue_.c_str(), 1);
        else
            unsetenv(name_.c_str());
    }

private:
    std::string name_;
    bool hadPreviousValue_;
    std::string previousValue_;
};

Export::VtkGrid makeSingleTriangle()
{
    Export::VtkGrid grid;
    grid.points = {Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.0, 0.0), Point3D(0.0, 1.0, 0.0)};
    grid.nodeIds = {0, 1, 2};
    grid.cells.push_back({Export::VtkCellType::Triangle, {0, 1, 2}});
    grid.cellFields.push_back({"Marker", std::vector<int>{7}});
    return grid;
}

} // namespace

class MeshSnapshotWriterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // ctest runs each test in its own process, in parallel.
        auto testName = std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
        directory_ = std::filesystem::temp_directory_path() / ("openloom_mesh_snapshot_writer_" + testName);
        std::filesystem::remove_all(directory_);
        std::filesystem::create_directories(directory_);
        unsetenv("EXPORT_MESH_EACH_ITERATION");
        unsetenv("EXPORT_MESH_EVERY_N");
        setenv("EXPORT_MESH_DIRECTORY", directory_.c_str(), 1);
        MeshSnapshotWriter::restartSequence();
    }

    void TearDown() override
    {
        unsetenv("EXPORT_MESH_DIRECTORY");
        std::filesystem::remove_all(directory_);
    }

    size_t countFiles() const
    {
        return static_cast<size_t>(std::distance(std::filesystem::directory_iterator(directory_),
                                                 std::filesystem::directory_iterator()));
    }

    bool exists(const std::string& name) const
    {
        return std::filesystem::exists(directory_ / name);
    }

    std::filesystem::path directory_;
};

TEST_F(MeshSnapshotWriterTest, WritesNothingWhenFlagUnset)
{
    MeshSnapshotWriter::write("seeded", makeSingleTriangle());

    EXPECT_FALSE(MeshSnapshotWriter::isEnabled());
    EXPECT_EQ(countFiles(), 0u);
}

TEST_F(MeshSnapshotWriterTest, WritesNumberedFilesInEveryFormat)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");
    auto grid = makeSingleTriangle();

    MeshSnapshotWriter::write("seeded", grid);
    MeshSnapshotWriter::write("refined", grid);

    EXPECT_TRUE(exists("rcdt_000000_seeded.vtu"));
    EXPECT_TRUE(exists("rcdt_000000_seeded.nodes.tsv"));
    EXPECT_TRUE(exists("rcdt_000000_seeded.cells.tsv"));
    EXPECT_TRUE(exists("rcdt_000001_refined.vtu"));
    EXPECT_TRUE(exists("rcdt_000001_refined.nodes.tsv"));
    EXPECT_TRUE(exists("rcdt_000001_refined.cells.tsv"));
}

TEST_F(MeshSnapshotWriterTest, InsertionStrideFollowsEnvironment)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");
    ScopedEnvironmentOverride stride("EXPORT_MESH_EVERY_N", "3");
    EXPECT_TRUE(MeshSnapshotWriter::isInsertionDue(0));
    EXPECT_FALSE(MeshSnapshotWriter::isInsertionDue(1));
    EXPECT_FALSE(MeshSnapshotWriter::isInsertionDue(2));
    EXPECT_TRUE(MeshSnapshotWriter::isInsertionDue(3));
    EXPECT_FALSE(MeshSnapshotWriter::isInsertionDue(5));
    EXPECT_TRUE(MeshSnapshotWriter::isInsertionDue(6));
}

TEST_F(MeshSnapshotWriterTest, InvalidStrideFallsBackToOne)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");

    for (auto invalid : {"0", "abc", "-2"})
    {
        ScopedEnvironmentOverride stride("EXPORT_MESH_EVERY_N", invalid);
        EXPECT_TRUE(MeshSnapshotWriter::isInsertionDue(1)) << invalid;
        EXPECT_TRUE(MeshSnapshotWriter::isInsertionDue(2)) << invalid;
    }
}

TEST_F(MeshSnapshotWriterTest, NoInsertionIsDueWhenFlagUnset)
{
    EXPECT_FALSE(MeshSnapshotWriter::isInsertionDue(0));
}

TEST_F(MeshSnapshotWriterTest, RestartSequenceNumbersFromZeroAgain)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");
    auto grid = makeSingleTriangle();

    MeshSnapshotWriter::write("first", grid);
    MeshSnapshotWriter::restartSequence();
    MeshSnapshotWriter::write("second", grid);

    EXPECT_TRUE(exists("rcdt_000000_first.vtu"));
    EXPECT_TRUE(exists("rcdt_000000_second.vtu"));
}

TEST_F(MeshSnapshotWriterTest, TsvCarriesTheGridFields)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");

    MeshSnapshotWriter::write("grid", makeSingleTriangle());

    std::ifstream cells(directory_ / "rcdt_000000_grid.cells.tsv");
    std::string header;
    std::string row;
    std::getline(cells, header);
    std::getline(cells, row);
    EXPECT_EQ(header, "kind\tid\tnodes\tMarker");
    EXPECT_EQ(row, "triangle\t0\t0,1,2\t7");
}
