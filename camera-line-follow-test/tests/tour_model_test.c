#include <assert.h>
#include <stdio.h>
#include "digit_model.h"
static digit_model_workspace_t w;
int main(void) {float error;int n=digit_model_selftest(&w,&error);printf("MODEL: %d/10 error=%g\n",n,error);assert(n==10 && error<0.002f);}
