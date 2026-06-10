#ifndef ASSERT_MACRO_H
#define ASSERT_MACRO_H

#define ASSERT(condition) (void)(condition)

#define BUILD_ASSERT(condition, message) \
    _Static_assert(condition, message)

#endif
