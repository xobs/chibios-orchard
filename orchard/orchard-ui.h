#ifndef __ORCHARD_UI_H__
#define __ORCHARD_UI_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "orchard.h"
#include "orchard-events.h"
#include "gfx.h"

struct _OrchardUi;
typedef struct _OrchardUi OrchardUi;
struct _OrchardAppContext;
typedef struct _OrchardAppContext OrchardAppContext;

// used to define lists of items for passing to UI elements
// for selection lists, the definition is straightfoward
// for dialog boxes, the first list item is the message; the next 1 or 2 items are button texts
// lists are linked lists, as we don't know a priori how long the lists will be
typedef struct OrchardUiContext {
  unsigned int selected;
  unsigned int total;
  char **itemlist;
} OrchardUiContext;

typedef struct _OrchardUi {
  char *name;
  void (*start)(OrchardAppContext *ui);
  void (*event)(OrchardAppContext *ui, const OrchardAppEvent *event);
  void (*exit)(OrchardAppContext *ui);
} OrchardUi;

#define orchard_ui_start()                                                   \
({                                                                            \
  static char start[0] __attribute__((unused,                                 \
    aligned(4), section(".chibi_list_ui_1")));                               \
  (const OrchardUi *)&start;                                                 \
})

#define orchard_ui(_name, _start, _event, _exit)                        \
  const OrchardUi _orchard_ui_list_##_start##_event##_exit           \
  __attribute__((unused, aligned(4), section(".chibi_list_ui_2_" # _start # _event # _exit))) =  \
     { _name, _start, _event, _exit }

#define orchard_ui_end()                                                     \
  const OrchardUi _orchard_ui_list_final                                    \
  __attribute__((unused, aligned(4), section(".chibi_list_ui_3_end"))) =     \
     { NULL, NULL, NULL, NULL }


#define TEXTENTRY_MAXLEN  15  // maximum length of any entered text, not including null char

const OrchardUi *getUiByName(const char *name);

void uiStart(void);
void orchardGfxStart(void);
void orchardGfxEnd(void);

#endif /* __ORCHARD_UI_H__ */
