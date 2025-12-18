#pragma once
#include "MeshTransaction.h"

namespace Meshing
{

class ScopedTransaction
{
public:
    explicit ScopedTransaction(MeshMutator3D* operations);
    ~ScopedTransaction();
    void commit();

private:
    MeshTransaction transaction_;
};

} // namespace Meshing
