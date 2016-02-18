#ifndef __MEMFUNC_H__
#define __MEMFUNC_H__

/* memfunc -- Copy code into RAM and run it from there.

  To use, you must know how large your functions are.  From ASM, define
  your functions like this:

.func funcname
.global funcname
funcname:
   ... function goes here ...
.type funcname, %function
.size funcname, .-funcname
.endfunc
.global funcname_size
.align 4
funcname_size: .long funcname, .-funcname

  Then, you can copy your function into RAM and call it:

  memfunc_load(funcname, symbol_to_define_on_stack);
  memfunc_call(symbol_to_define_on_stack, arg1, arg2, ...);

  'symbol_to_define_on_stack' will allocate a section of stack large
  enough to hold the function.  It will then copy the function to this
  area of stack.

  memfunc_call() takes a variable number of parameters and calls the
  function at the specified memory location.
*/

#define memfunc_size(x) ((uint32_t)((uint32_t )x ## _size))
#define memfunc_load(func, ramname) \
  extern int *func ## _size; \
  uint32_t ramname[memfunc_size(func) / 4]; \
  do { \
    unsigned int i; \
    uint32_t *ptr = (uint32_t *)(((uint32_t)func) & ~1); \
    for (i = 0; i < sizeof(ramname) / 4; i++) \
      ramname[i] = ptr[i]; \
  } while(0)

#define memfunc_GET_MACRO(_0,_1,_2,_3,_4,_5,_6,_7,_8_,_9,NAME,...) NAME
#define memfunc_call(...) memfunc_GET_MACRO(_0, ##__VA_ARGS__, memfunc_call9, memfunc_call8, memfunc_call7, memfunc_call6, memfunc_call5, memfunc_call4, memfunc_call3, memfunc_call2, memfunc_call1, memfunc_call0)(__VA_ARGS__)

#define memfunc_call5(name, a1, a2, a3, a4) \
  do { \
    void (*tmp)(uint32_t, uint32_t, uint32_t, uint32_t) = ((void *)(((uint32_t)name) + 1)); \
    tmp((uint32_t)a1, (uint32_t)a2, (uint32_t)a3, (uint32_t)a4); \
  } while(0);

#define memfunc_call4(name, a1, a2, a3) \
  do { \
    void (*tmp)(uint32_t, uint32_t, uint32_t) = ((void *)(((uint32_t)name) + 1)); \
    tmp((uint32_t)a1, (uint32_t)a2, (uint32_t)a3); \
  } while(0);

#define memfunc_call3(name, a1, a2) \
  do { \
    void (*tmp)(uint32_t, uint32_t) = ((void *)(((uint32_t)name) + 1)); \
    tmp((uint32_t)a1, (uint32_t)a2); \
  } while(0);

#define memfunc_call2(name, a1) \
  do { \
    void (*tmp)(uint32_t) = ((void *)(((uint32_t)name) + 1)); \
    tmp((uint32_t)a1); \
  } while(0);

#define memfunc_call1(name) \
  do { \
    void (*tmp)(void) = ((void *)(((uint32_t)name) + 1)); \
    tmp(); \
  } while(0);

#endif /* __MEMFUNC_H__ */
