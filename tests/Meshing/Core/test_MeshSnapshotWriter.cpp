#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"

#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <cstdlib>
#include <filesystem>
#include <string>

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

SurfaceMesh3D makeSingleTriangle()
{
    SurfaceMesh3D surfaceMesh;
    surfaceMesh.nodes = {Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.0, 0.0), Point3D(0.0, 1.0, 0.0)};
    surfaceMesh.triangles = {{0, 1, 2}};
    surfaceMesh.faceTriangleIds["surface"] = {0};
    return surfaceMesh;
}

} // namespace

class MeshSnapshotWriterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        directory_ = std::filesystem::temp_directory_path() / "openloom_mesh_snapshot_writer_test";
        std::filesystem::remove_all(directory_);
        std::filesystem::create_directories(directory_);
        unsetenv("EXPORT_MESH_EACH_ITERATION");
        unsetenv("EXPORT_MESH_EVERY_N");
    }

    void TearDown() override
    {
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
    MeshSnapshotWriter writer(directory_.string());

    writer.write("seeded", makeSingleTriangle());

    EXPECT_FALSE(writer.isEnabled());
    EXPECT_EQ(countFiles(), 0u);
}

TEST_F(MeshSnapshotWriterTest, WritesNumberedFilesInEveryFormat)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");
    MeshSnapshotWriter writer(directory_.string());
    auto surfaceMesh = makeSingleTriangle();

    writer.write("seeded", surfaceMesh);
    writer.write("refined", surfaceMesh);

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
    MeshSnapshotWriter writer(directory_.string());

    EXPECT_TRUE(writer.isInsertionDue(0));
    EXPECT_FALSE(writer.isInsertionDue(1));
    EXPECT_FALSE(writer.isInsertionDue(2));
    EXPECT_TRUE(writer.isInsertionDue(3));
    EXPECT_FALSE(writer.isInsertionDue(5));
    EXPECT_TRUE(writer.isInsertionDue(6));
}

TEST_F(MeshSnapshotWriterTest, InvalidStrideFallsBackToOne)
{
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");

    for (auto invalid : {"0", "abc", "-2"})
    {
        ScopedEnvironmentOverride stride("EXPORT_MESH_EVERY_N", invalid);
        MeshSnapshotWriter writer(directory_.string());

        EXPECT_TRUE(writer.isInsertionDue(1)) << invalid;
        EXPECT_TRUE(writer.isInsertionDue(2)) << invalid;
    }
}

TEST_F(MeshSnapshotWriterTest, NoInsertionIsDueWhenFlagUnset)
{
    MeshSnapshotWriter writer(directory_.string());

    EXPECT_FALSE(writer.isInsertionDue(0));
}
