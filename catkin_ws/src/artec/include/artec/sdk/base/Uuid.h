#pragma once

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IString.h>

namespace artec { namespace sdk { namespace base 
{

/// Unique identifier
class Uuid;

extern "C"
{
    /**
    * @brief Generate new UUID
    * @param uuid pointer to UUID to be created.
    * @return Error code.
    */
    ErrorCode ABASESDK_LINK_SPEC generateUuid(Uuid* uuid);

    /**
    * @brief Convert Uuid to string
    */
    ErrorCode ABASESDK_LINK_SPEC convertUuidtoString(IString** string, const Uuid* uuid);

    /**
    * @brief Convert string to Uuid
    */
    ErrorCode ABASESDK_LINK_SPEC converStringtoUuid(Uuid* uuid, IString* string);
}

class Uuid
{
public:
    enum { UuidSize = 16 };

    /**
     * @brief Create empty UUID {00000000-0000-0000-0000-000000000000}
     */
    Uuid()
    {
        for (int i = 0; i < UuidSize; ++i)
        {
            data_[i] = 0;
        }
    }

    /**
    * @brief Compare two UUIDs
    */
    static bool less(const Uuid& left, const Uuid& right)
    {
        for (size_t i = 0; i < Uuid::UuidSize; ++i)
        {
            if (left.data_[i] == right.data_[i])
            {
                continue;
            }
            return left.data_[i] < right.data_[i];
        }
        return false;
    }

    friend ErrorCode generateUuid(Uuid* uuid);
    friend ErrorCode convertUuidtoString(IString** string, const Uuid* uuid);
    friend ErrorCode converStringtoUuid(Uuid* uuid, IString* string);

private:
    unsigned char data_[UuidSize];
};

} } } 