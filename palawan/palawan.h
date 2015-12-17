#ifndef __PALAWAN_H__
#define __PALAWAN_H__

extern const char *gitversion;
extern struct evt_table orchard_events;

#define PALAWAN_OS_VERSION_MAJOR      1
#define PALAWAN_OS_VERSION_MINOR      0

#define serialDriver                  (&SD3)
#define stream_driver                 ((BaseSequentialStream *)serialDriver)
extern void *stream;

#define radioDriver                   (&KRADIO1)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
#endif

#endif /* __PALAWAN_H__ */
