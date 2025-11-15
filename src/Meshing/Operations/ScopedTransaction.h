#pragma once
#include "MeshTransaction.h"

namespace Meshing
{

class ScopedTransaction
{
public:
    explicit ScopedTransaction(MeshOperations* operations);
    ~ScopedTransaction();
    void commit();

private:
    MeshTransaction transaction_;
};

} // namespace Meshing
