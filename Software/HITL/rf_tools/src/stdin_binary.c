/* Linked into hackrf_transfer.exe: Windows opens stdin in text mode, where a
   0x1A byte reads as end of file, so "-t -" (IQ samples on stdin) would stop
   at the first 0x1A. Switch stdin to binary before main() runs. */
#include <fcntl.h>
#include <io.h>
#include <stdio.h>
__attribute__((constructor)) static void stdin_binary(void) { _setmode(_fileno(stdin), _O_BINARY); }
