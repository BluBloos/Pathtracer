/*
before including this file,
define TYPE and MAX_ELEMENTS
*/

#define NAME_GENERATOR_1(a,b) a##b
#define NAME_GENERATOR_2(a,b) NAME_GENERATOR_1(a,b)
#define NAME_GENERATOR(b) NAME_GENERATOR_2( TYPE, b)
#define NAME_GENERATOR3(a,b) NAME_GENERATOR_2(a, TYPE) ## b

typedef struct {
    int max_elements;
    int count;
    TYPE elements[MAX_ELEMENTS];
} NAME_GENERATOR( _array_t );

NAME_GENERATOR( _array_t )
NAME_GENERATOR3( create_, _array )()
{
    NAME_GENERATOR( _array_t ) result;
    result.count = 0;
    result.max_elements = MAX_ELEMENTS;
    return result;
}

#undef TYPE
#undef MAX_ELEMENTS
#undef NAME_GENERATOR
#undef NAME_GENERATOR3
#undef NAME_GENERATOR_1
#undef NAME_GENERATOR_2