https://stackoverflow.com/questions/1433204/how-do-i-use-extern-to-share-variables-between-source-files/1433387

The clean, reliable way to declare and define global variables is to use a header file to contain an extern declaration of the variable.

The header is included by the one source file that defines the variable and by all the source files that reference the variable. For each program, one source file (and only one source file) defines the variable. Similarly, one header file (and only one header file) should declare the variable. The header file is crucial; it enables cross-checking between independent TUs (translation units — think source files) and ensures consistency.

https://community.arm.com/support-forums/f/keil-forum/38565/extern-keyword-for-an-array/57074

cmd.h
```
#define LEN_CMD_FIRE 5 // Remember to count the NUL!
extern const char CMD_Fire[LEN_CMD_FIRE];
```

AAA.c
```
#include "cmd.h"

const char CMD_Fire[LEN_CMD_FIRE] = "FIRE";
```

BBB.c
```
#include "cmd.h"

// now you can use CMD_Fire and LEN_CMD_FIRE...
```