#include <stdlib.h>
#include <cassert>

// NOTE(Noah): Stretchy buffers adapated from the cryptic C code of https://nothings.org/stb/stretchy_buffer.txt
// Stretchy buffers basically work like so: A block of memory is allocated to store the current count, total element size,
// plus all the elements. The array pointer that was passed in originally is modified in place with the new element pointer,
// which is offset by 2 in the allocated block (just after the count and total element size).
// 
// All stretchy buffers must begin as a null pointer.

// Inits the stretchy buffer.
#define nc_sbinit(a)             (nc_sbGrow(a,1))
// Frees the strechy buffer. Warning: the array a will be a dangling pointer after this call.
#define nc_sbfree(a)             ((a) ? free(nc_sbGetMetadataPtr(a)), 0 : 0)
// Pushes a new element to the stretchy buffer.
#define nc_sbpush(a,v)           (nc_sbMaybeGrow(a,1), (a)[nc_sbGetCount(a)++] = (v))
// Returns a reference to the count of the stretchy buffer.
#define nc_sbcount(a)            ((a) ? nc_sbGetCount(a) : 0)
// Returns a reference to the last element of the stretchy buffer.
#define nc_sblast(a)             ((a)[nc_sbGetCount(a)-1])

// TODO: There was a bug in nc_sbpop. We ought to verify that this bug is not everywhere
// else in my code.
// Returns and deletes the last element from inside the stretchy buffer.
#define nc_sbpop(a)              ((a)[--nc_sbGetCount(a)])

#define nc_sbGetMetadataPtr(a)  ((int *) (a) - 2)
#define nc_sbGetBufferSize(a)   nc_sbGetMetadataPtr(a)[0]
#define nc_sbGetCount(a)        nc_sbGetMetadataPtr(a)[1]

#define nc_sbNeedGrow(a,n)      ((a) == 0 || nc_sbGetCount(a) + n >= nc_sbGetBufferSize(a))
#define nc_sbMaybeGrow(a,n)     (nc_sbNeedGrow(a,(n)) ? nc_sbGrow(a,n) : (void)0)
#define nc_sbGrow(a,n)          nc_sbGrowf((void **) &(a), (n), sizeof(*(a)))

void nc_sbGrowf(void **arr, int increment, int itemsize);

// STACK DATA STRUCTURE
// I admit that this is a very light wrapper around the stretchy buffer.
// maybe that won't be the case in the future.
template <typename Type>
struct stack_t
{
    Type *elements;//sb
};

template <typename Type>
int nc_ssize(stack_t<Type> &s)
{
    return nc_sbcount(s.elements);
}

template <typename Type>
Type &nc_spush(stack_t<Type> &s, Type elem)
{
    nc_sbpush(s.elements, elem);
    return nc_sblast(s.elements);
}

template <typename Type>
Type nc_spop(stack_t<Type> &s)
{
    return nc_sbpop(s.elements);
}
// STACK DATA STRUCTURE


#ifdef NC_DS_IMPLEMENTATION

void nc_sbGrowf(void **arr, int increment, int itemsize)
{
   int m = *arr ? 2 * nc_sbGetBufferSize(*arr) + increment : increment + 1;
   void *p = realloc(*arr ? nc_sbGetMetadataPtr(*arr) : 0, itemsize * m + sizeof(int) * 2);
   assert(p);
   if (p) {
      if (!*arr) ((int *) p)[1] = 0;
      *arr = (void *) ((int *) p + 2);
      nc_sbGetBufferSize(*arr) = m;
   }
}

#endif