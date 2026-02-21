#include "Common/Exceptions/Exception.h"
#include "Common/Exceptions/GeometryException.h"
#include "Common/Exceptions/MeshException.h"
#include "Common/Exceptions/TopologyException.h"
#include <gtest/gtest.h>

TEST(Exception, BasicException)
{
    try
    {
        OPENLOOM_THROW(OpenLoom::Exception, "Test error");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::Exception& e)
    {
        EXPECT_STREQ(e.typeName(), "OpenLoom::Exception");
        EXPECT_TRUE(std::string(e.what()).find("Test error") != std::string::npos);
        EXPECT_FALSE(e.location().empty());
        EXPECT_TRUE(e.location().find("test_Exception.cpp") != std::string::npos);
    }
}

TEST(Exception, ExceptionWithoutLocation)
{
    try
    {
        throw OpenLoom::Exception("Simple error");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::Exception& e)
    {
        EXPECT_STREQ(e.message().c_str(), "Simple error");
        EXPECT_TRUE(e.location().empty());
        EXPECT_STREQ(e.what(), "Simple error");
    }
}

TEST(GeometryException, BasicGeometryException)
{
    try
    {
        OPENLOOM_THROW_GEOMETRY("Geometry error");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::GeometryException& e)
    {
        EXPECT_STREQ(e.typeName(), "OpenLoom::GeometryException");
        EXPECT_EQ(e.code(), OpenLoom::GeometryException::ErrorCode::INVALID_GEOMETRY);
        EXPECT_EQ(e.errorCode(), 1001);
    }
}

TEST(GeometryException, EntityNotFound)
{
    try
    {
        throw OpenLoom::EntityNotFoundException("Surface", "123", "test.cpp:10");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::EntityNotFoundException& e)
    {
        EXPECT_EQ(e.getEntityType(), "Surface");
        EXPECT_EQ(e.getEntityId(), "123");
        EXPECT_EQ(e.code(), OpenLoom::GeometryException::ErrorCode::ENTITY_NOT_FOUND);
        EXPECT_EQ(e.errorCode(), 1000);
        EXPECT_TRUE(std::string(e.what()).find("Surface with ID '123' not found") != std::string::npos);
    }
}

TEST(GeometryException, EntityNotFoundMacro)
{
    try
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", "abc-456");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::EntityNotFoundException& e)
    {
        EXPECT_EQ(e.getEntityType(), "Edge");
        EXPECT_EQ(e.getEntityId(), "abc-456");
        EXPECT_FALSE(e.location().empty());
    }
}

TEST(GeometryException, NullGeometry)
{
    try
    {
        OPENLOOM_REQUIRE_NOT_NULL(nullptr, "TestPointer");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::NullGeometryException& e)
    {
        EXPECT_EQ(e.code(), OpenLoom::GeometryException::ErrorCode::NULL_POINTER);
        EXPECT_TRUE(std::string(e.what()).find("Null geometry pointer: TestPointer") != std::string::npos);
    }
}

TEST(MeshException, VerificationException)
{
    std::vector<std::string> errors = {"Error 1", "Error 2", "Error 3"};
    try
    {
        OPENLOOM_THROW_VERIFICATION_FAILED("Mesh is invalid", errors);
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::MeshVerificationException& e)
    {
        EXPECT_EQ(e.code(), OpenLoom::MeshException::ErrorCode::VERIFICATION_FAILED);
        EXPECT_EQ(e.errorCode(), 2001);
        EXPECT_EQ(e.getErrors().size(), 3);
        EXPECT_EQ(e.getErrors()[0], "Error 1");
        EXPECT_EQ(e.getErrors()[1], "Error 2");
        EXPECT_EQ(e.getErrors()[2], "Error 3");
    }
}

TEST(MeshException, MaxIterations)
{
    try
    {
        OPENLOOM_THROW_MAX_ITERATIONS("Mesh refinement", 100);
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::MaxIterationsException& e)
    {
        EXPECT_EQ(e.getOperation(), "Mesh refinement");
        EXPECT_EQ(e.getMaxIterations(), 100);
        EXPECT_EQ(e.code(), OpenLoom::MeshException::ErrorCode::MAX_ITERATIONS_REACHED);
        EXPECT_EQ(e.errorCode(), 2004);
        EXPECT_TRUE(std::string(e.what()).find("failed to converge after 100 iterations") != std::string::npos);
    }
}

TEST(MeshException, EntityNotFound)
{
    try
    {
        throw OpenLoom::MeshEntityNotFoundException("Node", 42, "mesh.cpp:100");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::MeshEntityNotFoundException& e)
    {
        EXPECT_EQ(e.getEntityType(), "Node");
        EXPECT_EQ(e.getEntityId(), 42);
        EXPECT_EQ(e.code(), OpenLoom::MeshException::ErrorCode::NODE_NOT_FOUND);
        EXPECT_TRUE(std::string(e.what()).find("Node with ID 42 not found") != std::string::npos);
    }
}

TEST(TopologyException, BasicTopologyException)
{
    try
    {
        OPENLOOM_THROW_TOPOLOGY(ENTITY_NOT_FOUND, "Topology entity missing");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::TopologyException& e)
    {
        EXPECT_STREQ(e.typeName(), "OpenLoom::TopologyException");
        EXPECT_EQ(e.code(), OpenLoom::TopologyException::ErrorCode::ENTITY_NOT_FOUND);
        EXPECT_EQ(e.errorCode(), 4000);
    }
}

TEST(Exception, InheritanceHierarchy)
{
    try
    {
        OPENLOOM_THROW_GEOMETRY("Test");
        FAIL();
    }
    catch (const OpenLoom::Exception& e)
    {
        EXPECT_TRUE(dynamic_cast<const OpenLoom::GeometryException*>(&e) != nullptr);
    }
}

TEST(Exception, RequireMacro)
{
    bool condition = true;
    EXPECT_NO_THROW(OPENLOOM_REQUIRE(condition, OpenLoom::Exception, "Should not throw"));

    condition = false;
    try
    {
        OPENLOOM_REQUIRE(condition, OpenLoom::Exception, "Condition failed");
        FAIL() << "Should have thrown";
    }
    catch (const OpenLoom::Exception& e)
    {
        EXPECT_TRUE(std::string(e.what()).find("Condition failed") != std::string::npos);
    }
}
