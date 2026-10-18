// Negative control of the post-link flash-layout check (cmake/flash_layout.cmake; decision 0053, ground-segment unit 14). A
// read-only block of MARV_PLANTED_FLASH_BYTES bytes (fill byte 0xa5: not the erased value, so it is real flash data), the
// flash below the config slots, is placed with the image's read-only data in flash; with the runtime's vectors and code
// before it the image ends inside slot A. The check must fail the build and remove the image.
#define MARV_STR2(x) #x
#define MARV_STR(x) MARV_STR2(x)

asm(".section .rodata.marv_planted,\"a\"\n"
    ".global marv_planted\n"
    "marv_planted:\n"
    ".fill " MARV_STR(MARV_PLANTED_FLASH_BYTES) ", 1, 0xa5\n"
    ".text\n");

extern "C" const unsigned char marv_planted[];

int main() {
  const volatile unsigned char* p = marv_planted;
  return p[0] == p[MARV_PLANTED_FLASH_BYTES - 1] ? 0 : 1;
}
