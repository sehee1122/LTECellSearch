#include "view.h"
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

struct stdoutput {
  view common;
  pthread_mutex_t lock;
};

static void clear(view *this)
{
  /* do nothing */
}

static void append(view *_this, char *s)
{
  struct stdoutput *this = (struct stdoutput *)_this;
  if (pthread_mutex_lock(&this->lock)) abort();
  printf("%s\n", s);
  if (pthread_mutex_unlock(&this->lock)) abort();
}

view *new_view_stdout(void)
{
  struct stdoutput *ret = calloc(1, sizeof(struct stdoutput));
  if (ret == NULL) abort();

  ret->common.clear = clear;
  ret->common.append = (void (*)(view *, ...))append;

  if (pthread_mutex_init(&ret->lock, NULL)) abort();

  return (view *)ret;
}
