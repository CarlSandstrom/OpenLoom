#include "ScopedTransaction.h"

namespace Meshing
{

ScopedTransaction::ScopedTransaction(MeshOperations* operations) :
    transaction_(operations)
{
    transaction_.begin();
}

ScopedTransaction::~ScopedTransaction()
{
    if (transaction_.isActive())
    {
        transaction_.rollback();
    }
}

void ScopedTransaction::commit()
{
    transaction_.commit();
}

} // namespace Meshing
