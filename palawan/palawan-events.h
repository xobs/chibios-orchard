#ifndef __PALAWAN_EVENTS__
#define __PALAWAN_EVENTS__

/* Palawan event wrappers.
   These simplify the ChibiOS eventing system.  To use, initialize the event
   table, hook an event, and then dispatch events.

  static void shell_termination_handler(eventid_t id) {
    chprintf(stream, "Shell exited.  Received id %d\r\n", id);
  }

  void main(int argc, char **argv) {
    struct evt_table events;

    // Support up to 8 events
    evtTableInit(events, 8);

    // Call shell_termination_handler() when 'shell_terminated' is emitted
    evtTableHook(events, shell_terminated, shell_termination_handler);

    // Dispatch all events
    while (TRUE)
      chEvtDispatch(evtHandlers(events), chEvtWaitOne(ALL_EVENTS));
   }
 */

struct evt_table {
  int size;
  int next;
  evhandler_t *handlers;
  event_listener_t *listeners;
};

#define evtTableInit(table, capacity)                                       \
  do {                                                                      \
    static evhandler_t handlers[capacity];                                  \
    static event_listener_t listeners[capacity];                            \
    table.size = capacity;                                                  \
    table.next = 0;                                                         \
    table.handlers = handlers;                                              \
    table.listeners = listeners;                                            \
  } while(0)

#define evtTableHook(table, event, callback)                                \
  do {                                                                      \
    if (CH_DBG_ENABLE_ASSERTS != FALSE)                                     \
      if (table.next >= table.size)                                         \
        chSysHalt("event table overflow");                                  \
    chEvtRegister(&event, &table.listeners[table.next], table.next);        \
    table.handlers[table.next] = callback;                                  \
    table.next++;                                                           \
  } while(0)

#define evtTableUnhook(table, event, callback)                              \
  do {                                                                      \
    int i;                                                                  \
    for (i = 0; i < table.next; i++) {                                      \
      if (table.handlers[i] == callback)                                    \
        chEvtUnregister(&event, &table.listeners[i]);                       \
    }                                                                       \
  } while(0)

#define evtHandlers(table)                                                  \
    table.handlers

#define evtListeners(table)                                                 \
    table.listeners

extern event_source_t ble_rdy;
extern event_source_t rf_pkt_rdy;
extern event_source_t gpiox_rdy;

// adc-related events
extern event_source_t celcius_rdy;
extern event_source_t mic_rdy;
extern event_source_t usbdet_rdy;

// BM radio protocol events
extern event_source_t radio_page;
extern event_source_t radio_sex;
extern event_source_t radio_app;

void palawanEventsStart(void);

/// Palawan App events

typedef enum _PalawanAppEventType {
  keyEvent,
  appEvent,
  timerEvent,
  uiEvent,
  adcEvent,
  radioEvent,
} PalawanAppEventType;

/* ------- */

typedef enum _PalawanAppEventKeyFlag {
  keyUp = 0,
  keyDown = 1,
} PalawanAppEventKeyFlag;

typedef enum _PalawanAppEventKeyCode {
  keyLeft = 0x80,
  keyRight = 0x81,
  keySelect = 0x82,
  keyCW = 0x83,
  keyCCW = 0x84,
} PalawanAppEventKeyCode;

typedef struct _PalawanUiEvent {
  uint8_t   code;
  uint8_t   flags;
} PalawanUiEvent;

typedef enum _PalawanUiEventCode {
  uiComplete = 0x01,
} PalawanUiEventCode;

typedef enum _PalawanUiEventFlags {
  uiOK = 0x01,
  uiCancel,
  uiError,
} PalawanUiEventFlags;

typedef struct _PalawanAdcEvent {
  uint8_t   code;
  uint8_t   flags;
} PalawanAdcEvent;

typedef enum _PalawanAdcEventCode {
  adcCodeTemp = 0x01,
  adcCodeMic,
  adcCodeUsbdet,
} PalawanAdcEventCode;

// note: no ADC flags yet

typedef struct _PalawanAppKeyEvent {
  uint8_t   code;
  uint8_t   flags;
} PalawanAppKeyEvent;

/* ------- */

typedef enum _PalawanAppLifeEventFlag {
  appStart,
  appTerminate,
} PalawanAppLifeEventFlag;

typedef struct _PalawanAppLifeEvent {
  uint8_t   event;
} PalawanAppLifeEvent;

/* ------- */

typedef struct _PalawanAppTimerEvent {
  uint32_t  usecs;
} PalawanAppTimerEvent;

/* ------- */

typedef struct _PalawanAppEvent {
  PalawanAppEventType     type;
  union {
    PalawanAppKeyEvent    key;
    PalawanAppLifeEvent   app;
    PalawanAppTimerEvent  timer;
    PalawanUiEvent        ui;
    PalawanAdcEvent       adc;
  };
} PalawanAppEvent;

#endif /* __PALAWAN_EVENTS__ */
