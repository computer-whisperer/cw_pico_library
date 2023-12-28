//
// Created by christian on 3/6/22.
//

#ifndef KALOGON_CUSHION_NANOPB_DECONSTRUCTION_MACROS_H
#define KALOGON_CUSHION_NANOPB_DECONSTRUCTION_MACROS_H

// Generic helper macros
#define CAT(a, ...) PRIMITIVE_CAT(a, __VA_ARGS__)
#define PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

#define QUOT(a) PRIMITIVE_QUOT(a)
#define PRIMITIVE_QUOT(a) #a

# define EMPTY(...)
# define DEFER(...) __VA_ARGS__ EMPTY()
# define OBSTRUCT(...) __VA_ARGS__ DEFER(EMPTY)()
# define EXPAND(...) __VA_ARGS__

#define ARRAY_MAX_COUNT(type, member) (sizeof(((type *)0)->member) / sizeof(((type *)0)->member[0]))

#define GET_FIRST_ARG(a, ...) a
#define GET_SECOND_ARG(a, b, ...) b
#define GET_THIRD_ARG(a, b, c, ...) c
#define GET_FORTH_ARG(a, b, c, d, ...) d

// Common name synthesizers
#define GET_MSGTYPE(parent_type, name) EXPAND(CAT(parent_type, CAT(_, CAT(name, _MSGTYPE))))
#define GET_ENUMTYPE(parent_type, name) EXPAND(CAT(parent_type, CAT(_, CAT(name, _ENUMTYPE))))
#define GET_ONEOF_NAME(name_data) CAT(GET_FIRST_ARG name_data, CAT(_, GET_SECOND_ARG name_data))

// basic ptypes
#define CLASSIFY_PTYPE_UINT64(basic, ...) basic
#define CLASSIFY_PTYPE_UINT32(basic, ...) basic
#define CLASSIFY_PTYPE_INT64(basic, ...) basic
#define CLASSIFY_PTYPE_INT32(basic, ...) basic
#define CLASSIFY_PTYPE_DOUBLE(basic, ...) basic
#define CLASSIFY_PTYPE_FLOAT(basic, ...) basic

// message type
#define CLASSIFY_PTYPE_MESSAGE(basic, message, ...) message

// enum type
#define CLASSIFY_PTYPE_UENUM(basic, message, enum, ...) enum

// bool type
#define CLASSIFY_PTYPE_BOOL(basic, message, enum, bool, ...) bool

// raw type
#define CLASSIFY_PTYPE_FIXED_LENGTH_BYTES(basic, message, enum, bool, raw, ...) raw

// str type
#define CLASSIFY_PTYPE_STRING(basic, message, enum, bool, raw, str) str
#define CLASSIFY_PTYPE_BYTES(basic, message, enum, bool, raw, str) str

#define CLASSIFY_PTYPE(type, ...) CAT(CLASSIFY_PTYPE_,type)(__VA_ARGS__)

#define CLASSIFY_FTYPE_SINGULAR(singular, ...) singular
#define CLASSIFY_FTYPE_ONEOF(singular, oneof, ...) oneof
#define CLASSIFY_FTYPE_OPTIONAL(singular, oneof, optional, ...) optional
#define CLASSIFY_FTYPE_REPEATED(singular, oneof, optional, repeated, ...) repeated

#define CLASSIFY_FTYPE(type, ...) CAT(CLASSIFY_FTYPE_,type)(__VA_ARGS__)

#define NANOPB_PARSE_NAME_DATA_SIMPLE(name_data)
#define NANOPB_PARSE_NAME_DATA_ONEOF(name_data)

#define NANOPB_PARSE_NAME_DATA(ftype, name_data) CLASSIFY_FTYPE(ftype, \
    EXPAND,                                                       \
    GET_ONEOF_NAME,                                                        \
    EXPAND,                                                       \
    EXPAND                                                        \
    )(name_data)

// The next block of macros is designed to find all fields of a given type referenced in a parent message specification. The core logic is duplicated thrice so that
// sub-sub-sub fields can be found, since c macros cannot be properly recursive. This means that the FIND_REFERENCED_TYPES system can handle up to 3 nested levels of messages.
// more levels can be added if the inner block is copied more times


// This is the root macro for finding referenced types in a tree of messages
#define FIND_REFERENCED_TYPES(X_MESSAGE, X_ENUM, namespace, parent_type) \
    CAT(parent_type,_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_0, ((X_MESSAGE, X_ENUM), namespace, parent_type))

// Output handlers when we find the particular referenced types
#define FIND_REFERENCED_TYPES_FOUND_ENUM(X_SET, namespace, parent_type, name) \
    GET_SECOND_ARG X_SET(GET_ENUMTYPE(parent_type, name), CAT(namespace, name))

#define FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name) \
    GET_FIRST_ARG X_SET(GET_MSGTYPE(parent_type, name), CAT(namespace, name))

// This block will get copied for however many levels of nested messages we want to handle
// copy 0

#define FIND_REFERENCED_TYPES_INNER_0(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_0_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                     \
    EMPTY,                                                                                      \
    EMPTY,                                                                                          \
    EMPTY                                                                                     \
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_0_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_1, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)

// copy 1

#define FIND_REFERENCED_TYPES_INNER_1(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_1_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                                                                           \
    EMPTY,                                                                                           \
    EMPTY,                                                                                      \
    EMPTY                                                                                     \
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_1_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_2, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)

// copy 2

#define FIND_REFERENCED_TYPES_INNER_2(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_2_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                                                                           \
    EMPTY,                                                                                           \
    EMPTY,                                                                                      \
    EMPTY \
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_2_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_3, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)

// copy 3

#define FIND_REFERENCED_TYPES_INNER_3(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_3_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                                                                           \
    EMPTY,                                                                                           \
    EMPTY,                                                                                      \
    EMPTY\
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_3_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_4, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)     \
                                                                                 \
// copy 4
#define FIND_REFERENCED_TYPES_INNER_4(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_4_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                                                                           \
    EMPTY,                                                                                           \
    EMPTY,                                                                                      \
    EMPTY\
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_4_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_5, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)

// copy 5
#define FIND_REFERENCED_TYPES_INNER_5(set, b, ftype, ptype, name_data, f) CLASSIFY_PTYPE(ptype, \
    EMPTY,                                                                                           \
    FIND_REFERENCED_TYPES_INNER_5_INNER,                                                           \
    FIND_REFERENCED_TYPES_FOUND_ENUM,                                                                                           \
    EMPTY,                                                                                           \
    EMPTY,                                                                                      \
    EMPTY\
    )(GET_FIRST_ARG set, GET_SECOND_ARG set, GET_THIRD_ARG set, NANOPB_PARSE_NAME_DATA(ftype, name_data))

#define FIND_REFERENCED_TYPES_INNER_5_INNER(X_SET, namespace, parent_type, name) \
    CAT(GET_MSGTYPE(parent_type, name),_FIELDLIST)(FIND_REFERENCED_TYPES_INNER_6, (X_SET, CAT(namespace, name), GET_MSGTYPE(parent_type, name)))        \
    FIND_REFERENCED_TYPES_FOUND_MESSAGE(X_SET, namespace, parent_type, name)

// Stub where copy 6 would go
#define FIND_REFERENCED_TYPES_INNER_6(X_SET, namespace, parent_type, name) FIND_REFERENCED_TYPES_INNER_6_NOT_IMPLEMENTED

#endif //KALOGON_CUSHION_NANOPB_DECONSTRUCTION_MACROS_H
