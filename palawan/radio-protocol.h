#ifndef __RADIO_PROTOCOL_H__
#define __RADIO_PROTOCOL_H__

#include <stdint.h>

typedef int (*radio_handler_t)(uint8_t port,
                               uint8_t src,
                               uint8_t dst,
                               uint8_t length,
                               const void *data);

typedef struct _RadioProtocol {
  radio_handler_t handler;
  const char *name;
  uint8_t port;
} RadioProtocol;

#define radio_protocol_start() \
({ \
  static char start[0] __attribute__((unused,  \
    aligned(4), section(".chibi_list_radio_protocol_1")));        \
  (const RadioProtocol *)&start;            \
})

#define radio_protocol(_name, _port, _func) \
  const RadioProtocol _radio_protocol_list_##_func \
  __attribute__((unused, aligned(4), section(".chibi_list_radio_protocol_2_" _name))) = \
     { _func, _name, _port }

#define radio_protocol_end() \
  const RadioProtocol _radio_protocol_list_##_func \
  __attribute__((unused, aligned(4), section(".chibi_list_radio_protocol_3_end"))) = \
     { NULL, NULL, 0 }

/* When we get a packet for an unregistered handler, call this function */
void radioSetDefaultHandler(radio_handler_t handler);

/* Pass a packet in here to have it processed */
void radioPacketDispatch(uint8_t port, uint8_t src, uint8_t dst,
                         uint32_t len, const void *data);

#endif /* __RADIO_PROTOCOL_H__ */
