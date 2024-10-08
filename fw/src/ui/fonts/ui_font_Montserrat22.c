/*******************************************************************************
 * Size: 22 px
 * Bpp: 1
 * Opts: --bpp 1 --size 22 --font D:/ProjektiOtvoreni/CydTermostat/sw/assets/fonts/Montserrat-MediumItalic.ttf -o D:/ProjektiOtvoreni/CydTermostat/sw/assets/fonts\ui_font_Montserrat22.c --format lvgl -r 0x20-0x7f --no-compress --no-prefilter
 ******************************************************************************/

#include "../ui.h"

#ifndef UI_FONT_MONTSERRAT22
#define UI_FONT_MONTSERRAT22 1
#endif

#if UI_FONT_MONTSERRAT22

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0x19, 0xce, 0x63, 0x18, 0xcc, 0x63, 0x18, 0x0,
    0x63, 0x18,

    /* U+0022 "\"" */
    0x66, 0xcd, 0x9a, 0x24, 0xd9, 0x80,

    /* U+0023 "#" */
    0x1, 0xc, 0x3, 0xc, 0x3, 0x8, 0x2, 0x18,
    0x3f, 0xff, 0x3f, 0xff, 0x6, 0x10, 0x4, 0x30,
    0xc, 0x30, 0xc, 0x20, 0xff, 0xfc, 0xff, 0xfc,
    0x18, 0x40, 0x10, 0xc0, 0x30, 0xc0, 0x30, 0x80,

    /* U+0024 "$" */
    0x0, 0x40, 0x3, 0x0, 0x7f, 0x3, 0xff, 0x1c,
    0x88, 0x62, 0x3, 0x98, 0xe, 0x60, 0x1d, 0x0,
    0x3e, 0x0, 0x3e, 0x0, 0xfc, 0x3, 0x30, 0x8,
    0xc0, 0x23, 0x38, 0x9c, 0x7f, 0xe0, 0x7e, 0x0,
    0x40, 0x1, 0x0,

    /* U+0025 "%" */
    0x3c, 0x6, 0xcc, 0x1b, 0x8, 0x66, 0x10, 0x88,
    0x22, 0x10, 0xcc, 0x21, 0xb0, 0x66, 0xc0, 0x7b,
    0x3c, 0xc, 0xcc, 0x11, 0x8, 0x66, 0x11, 0x8c,
    0x26, 0x18, 0xd8, 0x31, 0x60, 0x3c,

    /* U+0026 "&" */
    0x7, 0xc0, 0x7f, 0x7, 0x18, 0x30, 0xc1, 0x86,
    0xc, 0x60, 0x3e, 0x1, 0xc0, 0x3f, 0x3, 0x1c,
    0xf0, 0x77, 0x81, 0xec, 0x6, 0x70, 0xfb, 0xfe,
    0xe7, 0xc2,

    /* U+0027 "'" */
    0x6d, 0xa5, 0x80,

    /* U+0028 "(" */
    0xc, 0x18, 0x61, 0xc3, 0xe, 0x18, 0x30, 0x61,
    0x83, 0x6, 0xc, 0x18, 0x30, 0x60, 0xc1, 0x83,
    0x6, 0x6, 0x0,

    /* U+0029 ")" */
    0xc, 0x18, 0x30, 0x60, 0x60, 0xc1, 0x83, 0xe,
    0x18, 0x30, 0x60, 0xc3, 0x86, 0xc, 0x30, 0x61,
    0x83, 0xc, 0x0,

    /* U+002A "*" */
    0x4, 0x2, 0x1b, 0x67, 0xe1, 0xc3, 0xf9, 0x24,
    0x30, 0x18, 0x0,

    /* U+002B "+" */
    0x6, 0x3, 0x0, 0xc0, 0x30, 0xff, 0xff, 0xf0,
    0x80, 0x60, 0x18, 0x6, 0x0,

    /* U+002C "," */
    0x7, 0x73, 0x66, 0x40,

    /* U+002D "-" */
    0xff, 0xf0,

    /* U+002E "." */
    0x7f, 0x80,

    /* U+002F "/" */
    0x0, 0x18, 0x1, 0x80, 0xc, 0x0, 0xc0, 0x6,
    0x0, 0x60, 0x3, 0x0, 0x30, 0x3, 0x80, 0x18,
    0x1, 0x80, 0xc, 0x0, 0xc0, 0x6, 0x0, 0x60,
    0x3, 0x0, 0x30, 0x1, 0x80, 0x18, 0x0, 0xc0,
    0xc, 0x0, 0x0,

    /* U+0030 "0" */
    0x7, 0xc0, 0x7f, 0x87, 0xc, 0x70, 0x77, 0x1,
    0xb0, 0xd, 0x80, 0x7c, 0x3, 0xe0, 0x1e, 0x1,
    0xf0, 0xd, 0xc0, 0xee, 0x6, 0x38, 0xf0, 0xff,
    0x3, 0xe0,

    /* U+0031 "1" */
    0xff, 0xe1, 0x86, 0x18, 0xe3, 0x8c, 0x30, 0xc3,
    0x1c, 0x61, 0x86, 0x18,

    /* U+0032 "2" */
    0x7, 0xc1, 0xfe, 0x38, 0x70, 0x3, 0x0, 0x30,
    0x3, 0x0, 0x70, 0xe, 0x1, 0xc0, 0x38, 0x7,
    0x0, 0xc0, 0x38, 0x7, 0x0, 0xff, 0xef, 0xfe,

    /* U+0033 "3" */
    0xf, 0xfc, 0x7f, 0xf0, 0x3, 0x80, 0x1c, 0x0,
    0xe0, 0x7, 0x0, 0x38, 0x0, 0xf8, 0x1, 0xf0,
    0x0, 0xc0, 0x3, 0x0, 0xc, 0x40, 0x31, 0x83,
    0xc7, 0xfe, 0x7, 0xe0,

    /* U+0034 "4" */
    0x0, 0x38, 0x1, 0xc0, 0xe, 0x0, 0x70, 0x3,
    0x80, 0x1c, 0x0, 0xe0, 0x7, 0xc, 0x18, 0x30,
    0xc0, 0xc7, 0xff, 0xdf, 0xff, 0x0, 0x60, 0x1,
    0x80, 0x6, 0x0, 0x30,

    /* U+0035 "5" */
    0xf, 0xf8, 0x7f, 0xc7, 0x0, 0x30, 0x1, 0x80,
    0xc, 0x0, 0xfe, 0x7, 0xfc, 0x0, 0xe0, 0x3,
    0x80, 0x1c, 0x0, 0xe0, 0x6, 0x70, 0x73, 0xff,
    0x7, 0xe0,

    /* U+0036 "6" */
    0x3, 0xf0, 0x7f, 0x87, 0x4, 0x70, 0x7, 0x0,
    0x30, 0x1, 0xbe, 0x1f, 0xfc, 0xf0, 0xe7, 0x3,
    0xb8, 0xd, 0xc0, 0x66, 0x6, 0x38, 0x70, 0xff,
    0x3, 0xf0,

    /* U+0037 "7" */
    0xff, 0xff, 0xff, 0xc0, 0x6c, 0xe, 0xc0, 0xc0,
    0x18, 0x3, 0x80, 0x30, 0x7, 0x0, 0x60, 0xc,
    0x1, 0xc0, 0x18, 0x3, 0x80, 0x70, 0x6, 0x0,

    /* U+0038 "8" */
    0x7, 0xc0, 0xff, 0x8e, 0x1c, 0x60, 0x73, 0x1,
    0x98, 0x1c, 0xe1, 0xc3, 0xfc, 0x7f, 0xc7, 0x7,
    0x30, 0xd, 0x80, 0x6c, 0x7, 0x70, 0x71, 0xff,
    0x7, 0xf0,

    /* U+0039 "9" */
    0xf, 0x83, 0xfc, 0x70, 0xe6, 0x7, 0x60, 0x36,
    0x3, 0x60, 0x77, 0xf, 0x3f, 0xb1, 0xf7, 0x0,
    0x60, 0x6, 0x0, 0xc8, 0x38, 0xff, 0xf, 0xc0,

    /* U+003A ":" */
    0x39, 0xcc, 0x0, 0x0, 0x0, 0x3, 0x39, 0xc0,

    /* U+003B ";" */
    0x1c, 0x71, 0x80, 0x0, 0x0, 0x0, 0x0, 0xc7,
    0x1c, 0x21, 0x84, 0x30,

    /* U+003C "<" */
    0x0, 0x20, 0x38, 0x1e, 0x1f, 0xf, 0x1, 0x80,
    0x1e, 0x0, 0xf0, 0x7, 0x80, 0x30,

    /* U+003D "=" */
    0x7f, 0xef, 0xfc, 0x0, 0x0, 0x0, 0x1, 0xff,
    0xbf, 0xf0,

    /* U+003E ">" */
    0x20, 0x7, 0x0, 0x78, 0x3, 0xc0, 0x1e, 0x1,
    0xc1, 0xe1, 0xf0, 0xf0, 0x10, 0x0,

    /* U+003F "?" */
    0x1f, 0x1f, 0xfe, 0x1c, 0x3, 0x0, 0xc0, 0x30,
    0x18, 0x1c, 0xe, 0x7, 0x1, 0x80, 0x0, 0x0,
    0xc, 0x3, 0x80, 0xc0,

    /* U+0040 "@" */
    0x0, 0xfe, 0x0, 0x1f, 0xfe, 0x3, 0xc0, 0x78,
    0x38, 0x0, 0xe3, 0x8f, 0x9b, 0x18, 0xfe, 0xc9,
    0x8e, 0x1c, 0x68, 0xe0, 0x63, 0xc6, 0x3, 0x1e,
    0x30, 0x18, 0xf3, 0x0, 0xc7, 0x18, 0x6, 0x2c,
    0x60, 0x63, 0x63, 0x87, 0x13, 0xf, 0xff, 0x8c,
    0x3e, 0x78, 0x70, 0x0, 0x1, 0xe0, 0x0, 0x7,
    0xfe, 0x0, 0xf, 0xe0, 0x0,

    /* U+0041 "A" */
    0x0, 0x30, 0x0, 0x70, 0x0, 0xd8, 0x0, 0xd8,
    0x1, 0x98, 0x3, 0x98, 0x3, 0xc, 0x6, 0xc,
    0x6, 0xc, 0xc, 0xc, 0x1f, 0xfe, 0x1f, 0xfe,
    0x30, 0x6, 0x70, 0x6, 0x60, 0x7, 0xc0, 0x3,

    /* U+0042 "B" */
    0x1f, 0xf8, 0x3f, 0xf8, 0x60, 0x38, 0xc0, 0x33,
    0x80, 0x66, 0x1, 0xcc, 0x7, 0x1f, 0xfc, 0x3f,
    0xf8, 0xe0, 0x39, 0x80, 0x33, 0x0, 0x66, 0x0,
    0xcc, 0x7, 0xbf, 0xfe, 0x7f, 0xf0,

    /* U+0043 "C" */
    0x3, 0xf8, 0x1f, 0xfc, 0x78, 0x39, 0xc0, 0x3,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x70, 0x0, 0xe0,
    0x1, 0xc0, 0x3, 0x80, 0x3, 0x0, 0x7, 0x0,
    0x87, 0x7, 0x87, 0xfe, 0x7, 0xf0,

    /* U+0044 "D" */
    0x1f, 0xf0, 0xf, 0xfe, 0x6, 0x3, 0x83, 0x0,
    0xe3, 0x80, 0x31, 0x80, 0x1c, 0xc0, 0xe, 0x60,
    0x7, 0x30, 0x3, 0x38, 0x1, 0x98, 0x1, 0xcc,
    0x0, 0xc6, 0x0, 0xe3, 0x1, 0xe3, 0xff, 0xe1,
    0xff, 0xc0,

    /* U+0045 "E" */
    0x1f, 0xfc, 0x7f, 0xf1, 0x80, 0x6, 0x0, 0x38,
    0x0, 0xc0, 0x3, 0x0, 0xf, 0xfc, 0x3f, 0xf1,
    0xc0, 0x6, 0x0, 0x18, 0x0, 0x60, 0x1, 0x80,
    0xf, 0xff, 0x3f, 0xf8,

    /* U+0046 "F" */
    0x1f, 0xfc, 0x7f, 0xf1, 0x80, 0x6, 0x0, 0x38,
    0x0, 0xc0, 0x3, 0x0, 0xc, 0x0, 0x3f, 0xf1,
    0xff, 0x86, 0x0, 0x18, 0x0, 0x60, 0x1, 0x80,
    0xe, 0x0, 0x30, 0x0,

    /* U+0047 "G" */
    0x3, 0xf8, 0xf, 0xfe, 0x1e, 0xf, 0x38, 0x0,
    0x30, 0x0, 0x60, 0x0, 0x60, 0x0, 0xe0, 0x0,
    0xe0, 0xc, 0xe0, 0xc, 0xe0, 0xc, 0x60, 0xc,
    0x70, 0x1c, 0x38, 0x38, 0x1f, 0xf8, 0xf, 0xe0,

    /* U+0048 "H" */
    0x18, 0x3, 0x18, 0x3, 0x18, 0x3, 0x18, 0x3,
    0x38, 0x7, 0x30, 0x6, 0x30, 0x6, 0x3f, 0xfe,
    0x3f, 0xfe, 0x70, 0xe, 0x60, 0xc, 0x60, 0xc,
    0x60, 0xc, 0x60, 0xc, 0xe0, 0x1c, 0xc0, 0x1c,

    /* U+0049 "I" */
    0x18, 0xc6, 0x33, 0x98, 0xc6, 0x33, 0x98, 0xc6,
    0x33, 0x98,

    /* U+004A "J" */
    0xf, 0xf0, 0xff, 0x0, 0x70, 0x6, 0x0, 0x60,
    0x6, 0x0, 0x60, 0xe, 0x0, 0xc0, 0xc, 0x0,
    0xc0, 0xc, 0x41, 0x8e, 0x38, 0x7f, 0x3, 0xe0,

    /* U+004B "K" */
    0x18, 0x7, 0x18, 0xe, 0x18, 0x1c, 0x18, 0x38,
    0x38, 0x60, 0x31, 0xc0, 0x33, 0x80, 0x37, 0x0,
    0x3f, 0x80, 0x7d, 0x80, 0x78, 0xc0, 0x70, 0xe0,
    0x60, 0x60, 0x60, 0x30, 0xe0, 0x38, 0xc0, 0x18,

    /* U+004C "L" */
    0x18, 0x3, 0x0, 0x60, 0xc, 0x3, 0x80, 0x60,
    0xc, 0x1, 0x80, 0x30, 0xe, 0x1, 0x80, 0x30,
    0x6, 0x0, 0xc0, 0x3f, 0xff, 0xff,

    /* U+004D "M" */
    0x18, 0x0, 0x71, 0xc0, 0x6, 0x1c, 0x0, 0xe1,
    0xc0, 0x1e, 0x36, 0x3, 0xe3, 0x60, 0x3e, 0x36,
    0x6, 0xc3, 0x30, 0xcc, 0x33, 0x1c, 0xc7, 0x19,
    0x8c, 0x61, 0xb1, 0xc6, 0x1e, 0x18, 0x60, 0xe1,
    0x86, 0xc, 0x18, 0xe0, 0x1, 0x8c, 0x0, 0x18,

    /* U+004E "N" */
    0x18, 0x3, 0x1c, 0x3, 0x1c, 0x3, 0x1e, 0x3,
    0x3f, 0x7, 0x33, 0x6, 0x33, 0x86, 0x31, 0x86,
    0x31, 0xc6, 0x70, 0xce, 0x60, 0xec, 0x60, 0x6c,
    0x60, 0x3c, 0x60, 0x3c, 0xe0, 0x1c, 0xc0, 0x1c,

    /* U+004F "O" */
    0x3, 0xf8, 0x7, 0xff, 0x7, 0x83, 0xc7, 0x0,
    0x63, 0x0, 0x3b, 0x0, 0xd, 0x80, 0x7, 0xc0,
    0x3, 0xe0, 0x3, 0xf0, 0x1, 0xf8, 0x0, 0xcc,
    0x0, 0xe7, 0x0, 0xe1, 0xc1, 0xe0, 0x7f, 0xe0,
    0x1f, 0xc0,

    /* U+0050 "P" */
    0x1f, 0xf0, 0x3f, 0xf0, 0x60, 0x70, 0xc0, 0x63,
    0x80, 0xe6, 0x1, 0xcc, 0x3, 0x18, 0x6, 0x30,
    0x38, 0xff, 0xe1, 0xff, 0x83, 0x0, 0x6, 0x0,
    0xc, 0x0, 0x38, 0x0, 0x60, 0x0,

    /* U+0051 "Q" */
    0x3, 0xf8, 0x7, 0xff, 0x7, 0x83, 0xc7, 0x0,
    0x63, 0x0, 0x3b, 0x0, 0xd, 0x80, 0x7, 0xc0,
    0x3, 0xe0, 0x3, 0xf0, 0x1, 0xb8, 0x0, 0xcc,
    0x0, 0xe7, 0x0, 0xe1, 0xe1, 0xe0, 0x7f, 0xc0,
    0xf, 0x80, 0x1, 0xc2, 0x0, 0x7f, 0x0, 0x1f,
    0x0,

    /* U+0052 "R" */
    0x1f, 0xf0, 0x3f, 0xf0, 0x60, 0x70, 0xc0, 0x73,
    0x80, 0xe6, 0x1, 0xcc, 0x3, 0x18, 0x6, 0x30,
    0x38, 0xff, 0xe1, 0xff, 0x83, 0x6, 0x6, 0x6,
    0xc, 0xe, 0x38, 0xc, 0x60, 0x1c,

    /* U+0053 "S" */
    0x3, 0xf0, 0x3f, 0xf1, 0xc1, 0x86, 0x0, 0x38,
    0x0, 0xe0, 0x1, 0xc0, 0x3, 0xe0, 0x3, 0xe0,
    0x1, 0xc0, 0x3, 0x0, 0xc, 0x0, 0x33, 0x81,
    0xc7, 0xfe, 0x7, 0xe0,

    /* U+0054 "T" */
    0xff, 0xff, 0xff, 0x81, 0x80, 0xc, 0x0, 0xe0,
    0x6, 0x0, 0x30, 0x1, 0x80, 0xc, 0x0, 0xe0,
    0x6, 0x0, 0x30, 0x1, 0x80, 0xc, 0x0, 0xe0,
    0x6, 0x0,

    /* U+0055 "U" */
    0x30, 0x6, 0x60, 0xc, 0xc0, 0x3b, 0x80, 0x66,
    0x0, 0xcc, 0x1, 0x98, 0x3, 0x30, 0xe, 0xe0,
    0x19, 0x80, 0x33, 0x0, 0x66, 0x1, 0xce, 0x3,
    0xe, 0x1e, 0xf, 0xf8, 0xf, 0xc0,

    /* U+0056 "V" */
    0xc0, 0x6, 0xc0, 0xe, 0xe0, 0xc, 0x60, 0x18,
    0x60, 0x18, 0x60, 0x30, 0x70, 0x70, 0x30, 0x60,
    0x30, 0xc0, 0x31, 0xc0, 0x39, 0x80, 0x1b, 0x80,
    0x1b, 0x0, 0x1e, 0x0, 0x1e, 0x0, 0x1c, 0x0,

    /* U+0057 "W" */
    0xc0, 0x30, 0x7, 0x80, 0xf0, 0x1b, 0x1, 0xe0,
    0x76, 0x7, 0xc0, 0xcc, 0xd, 0x83, 0x98, 0x3b,
    0x6, 0x38, 0x66, 0x1c, 0x31, 0xcc, 0x30, 0x63,
    0x18, 0xe0, 0xcc, 0x19, 0x81, 0x98, 0x37, 0x3,
    0x60, 0x6c, 0x6, 0xc0, 0xf8, 0xf, 0x1, 0xe0,
    0x1e, 0x3, 0xc0, 0x18, 0x7, 0x0,

    /* U+0058 "X" */
    0xc, 0x3, 0x87, 0x3, 0x81, 0x83, 0x80, 0xe3,
    0x80, 0x31, 0x80, 0x1d, 0x80, 0x7, 0x80, 0x1,
    0x80, 0x1, 0xc0, 0x1, 0xf0, 0x1, 0xd8, 0x1,
    0xc6, 0x1, 0xc3, 0x1, 0xc0, 0xc1, 0xc0, 0x70,
    0xc0, 0x18,

    /* U+0059 "Y" */
    0xc0, 0xf, 0x80, 0x66, 0x3, 0x18, 0x1c, 0x30,
    0xe0, 0xc3, 0x3, 0x98, 0x6, 0xc0, 0x1f, 0x0,
    0x38, 0x0, 0xc0, 0x3, 0x0, 0x1c, 0x0, 0x70,
    0x1, 0x80, 0x6, 0x0,

    /* U+005A "Z" */
    0x1f, 0xfe, 0x3f, 0xfc, 0x0, 0x30, 0x0, 0xc0,
    0x3, 0x0, 0xc, 0x0, 0x30, 0x0, 0xc0, 0x3,
    0x80, 0xe, 0x0, 0x38, 0x0, 0xe0, 0x3, 0x80,
    0xe, 0x0, 0x3f, 0xfe, 0x7f, 0xfc,

    /* U+005B "[" */
    0xf, 0x87, 0xc3, 0x1, 0x81, 0xc0, 0xc0, 0x60,
    0x30, 0x18, 0x1c, 0xc, 0x6, 0x3, 0x1, 0x80,
    0xc0, 0xc0, 0x60, 0x30, 0x18, 0xf, 0xf, 0x80,

    /* U+005C "\\" */
    0xc6, 0x31, 0x8c, 0x21, 0x8c, 0x63, 0x18, 0xc2,
    0x18, 0xc6, 0x31, 0x84, 0x31, 0x80,

    /* U+005D "]" */
    0xf, 0x87, 0xc0, 0x60, 0x60, 0x30, 0x18, 0xc,
    0x6, 0x7, 0x3, 0x1, 0x80, 0xc0, 0x60, 0x70,
    0x30, 0x18, 0xc, 0x6, 0x7, 0xf, 0x7, 0x80,

    /* U+005E "^" */
    0x3, 0x1, 0xc0, 0x70, 0x34, 0x19, 0x6, 0x63,
    0x18, 0x82, 0x60, 0xb0, 0x30,

    /* U+005F "_" */
    0xff, 0xff, 0xfc,

    /* U+0060 "`" */
    0xe3, 0x8c,

    /* U+0061 "a" */
    0xf, 0x99, 0xfe, 0xdc, 0x1e, 0xc0, 0x6c, 0x3,
    0x60, 0x1b, 0x0, 0xd8, 0xe, 0xc0, 0x77, 0x7,
    0x1f, 0xf8, 0x3c, 0xc0,

    /* U+0062 "b" */
    0x18, 0x0, 0xc0, 0x6, 0x0, 0x70, 0x3, 0x0,
    0x19, 0xe0, 0xff, 0xc7, 0x7, 0x70, 0x1b, 0x0,
    0xd8, 0x6, 0xc0, 0x36, 0x1, 0xb0, 0x1b, 0xc1,
    0xdb, 0xfc, 0xcf, 0x80,

    /* U+0063 "c" */
    0xf, 0x83, 0xfe, 0x70, 0x66, 0x0, 0xc0, 0xc,
    0x0, 0xc0, 0xc, 0x0, 0xc0, 0xe, 0xc, 0x7f,
    0xc1, 0xf0,

    /* U+0064 "d" */
    0x0, 0xc, 0x0, 0x30, 0x0, 0xc0, 0x6, 0x0,
    0x18, 0x3e, 0x63, 0xfd, 0x9c, 0x1e, 0x60, 0x33,
    0x0, 0xcc, 0x3, 0x30, 0xc, 0xc0, 0x73, 0x1,
    0xce, 0xe, 0x1f, 0xf8, 0x1e, 0x60,

    /* U+0065 "e" */
    0xf, 0x83, 0xfc, 0x70, 0xe6, 0x6, 0xc0, 0x3f,
    0xff, 0xff, 0xfc, 0x0, 0xc0, 0xe, 0xc, 0x7f,
    0xc1, 0xf0,

    /* U+0066 "f" */
    0x7, 0x87, 0xc7, 0x3, 0x1, 0x87, 0xfb, 0xfc,
    0x60, 0x30, 0x18, 0x1c, 0xc, 0x6, 0x3, 0x1,
    0x81, 0xc0, 0xc0, 0x0,

    /* U+0067 "g" */
    0x7, 0xcc, 0x7f, 0xb3, 0x83, 0xcc, 0x7, 0x60,
    0x19, 0x80, 0x66, 0x1, 0x98, 0xe, 0x70, 0x78,
    0xff, 0xc0, 0xfb, 0x0, 0xc, 0x0, 0x71, 0x83,
    0x8f, 0xfc, 0xf, 0xc0,

    /* U+0068 "h" */
    0x18, 0x1, 0x80, 0x18, 0x3, 0x80, 0x30, 0x3,
    0x3c, 0x3f, 0xf3, 0x87, 0x70, 0x37, 0x3, 0x60,
    0x36, 0x3, 0x60, 0x36, 0x7, 0xc0, 0x6c, 0x6,
    0xc0, 0x60,

    /* U+0069 "i" */
    0x18, 0xc6, 0x0, 0x18, 0xc6, 0x73, 0x18, 0xc6,
    0x33, 0x18, 0xc0,

    /* U+006A "j" */
    0x0, 0xc0, 0x30, 0xc, 0x0, 0x0, 0x0, 0x60,
    0x18, 0x6, 0x1, 0x80, 0xc0, 0x30, 0xc, 0x3,
    0x0, 0xc0, 0x60, 0x18, 0x6, 0x1, 0x80, 0xe3,
    0xf0, 0x78, 0x0,

    /* U+006B "k" */
    0x18, 0x0, 0xc0, 0x6, 0x0, 0x70, 0x3, 0x0,
    0x18, 0x1c, 0xc1, 0xc6, 0x38, 0x73, 0x83, 0x38,
    0x1b, 0x80, 0xf6, 0x7, 0x38, 0x30, 0xc3, 0x7,
    0x18, 0x18, 0xc0, 0x60,

    /* U+006C "l" */
    0x18, 0xc6, 0x73, 0x18, 0xc6, 0x73, 0x18, 0xc6,
    0x33, 0x18, 0xc0,

    /* U+006D "m" */
    0x37, 0xc3, 0xe1, 0xff, 0x7f, 0x8e, 0x1f, 0xc,
    0xe0, 0x70, 0x77, 0x3, 0x3, 0xb0, 0x18, 0x19,
    0x80, 0xc0, 0xcc, 0xc, 0x6, 0x60, 0x60, 0x36,
    0x3, 0x1, 0xb0, 0x18, 0x1d, 0x80, 0xc0, 0xc0,

    /* U+006E "n" */
    0x33, 0xc3, 0xff, 0x38, 0x77, 0x3, 0x70, 0x36,
    0x3, 0x60, 0x36, 0x3, 0x60, 0x7c, 0x6, 0xc0,
    0x6c, 0x6,

    /* U+006F "o" */
    0xf, 0x83, 0xfe, 0x70, 0x76, 0x3, 0xc0, 0x3c,
    0x3, 0xc0, 0x3c, 0x7, 0xc0, 0x6e, 0xe, 0x7f,
    0xc1, 0xf0,

    /* U+0070 "p" */
    0x19, 0xe0, 0x7f, 0xe1, 0xc1, 0xce, 0x3, 0x30,
    0xc, 0xc0, 0x33, 0x0, 0xcc, 0x3, 0x70, 0x19,
    0xe0, 0xe6, 0xff, 0x19, 0xf0, 0x60, 0x3, 0x80,
    0xc, 0x0, 0x30, 0x0,

    /* U+0071 "q" */
    0xf, 0x99, 0xfe, 0xdc, 0x1e, 0xc0, 0x6c, 0x3,
    0x60, 0x1b, 0x0, 0xd8, 0xe, 0xc0, 0x67, 0x7,
    0x1f, 0xf8, 0x3c, 0xc0, 0x6, 0x0, 0x60, 0x3,
    0x0, 0x18,

    /* U+0072 "r" */
    0x33, 0x3f, 0x38, 0x70, 0x70, 0x60, 0x60, 0x60,
    0x60, 0xc0, 0xc0, 0xc0,

    /* U+0073 "s" */
    0xf, 0x87, 0xfc, 0xc1, 0x30, 0x7, 0x0, 0x7c,
    0x3, 0xe0, 0xe, 0x0, 0xd8, 0x3b, 0xfe, 0x1f,
    0x80,

    /* U+0074 "t" */
    0x18, 0x18, 0x18, 0xff, 0xff, 0x30, 0x30, 0x30,
    0x70, 0x60, 0x60, 0x60, 0x60, 0x7c, 0x3c,

    /* U+0075 "u" */
    0x60, 0x36, 0x3, 0x60, 0x7e, 0x6, 0xc0, 0x6c,
    0x6, 0xc0, 0x6c, 0xe, 0xc0, 0xce, 0x1c, 0xff,
    0xc7, 0xcc,

    /* U+0076 "v" */
    0xc0, 0x3e, 0x7, 0x60, 0x66, 0xc, 0x60, 0xc6,
    0x18, 0x33, 0x3, 0x30, 0x36, 0x3, 0xe0, 0x1c,
    0x1, 0x80,

    /* U+0077 "w" */
    0xc0, 0x60, 0x34, 0xe, 0x6, 0x60, 0xe0, 0x66,
    0x1b, 0xc, 0x61, 0xb1, 0x86, 0x33, 0x18, 0x63,
    0x33, 0x2, 0x63, 0x30, 0x3c, 0x1e, 0x3, 0xc1,
    0xe0, 0x38, 0x1c, 0x3, 0x81, 0xc0,

    /* U+0078 "x" */
    0x18, 0x18, 0x30, 0xc0, 0xc6, 0x1, 0xb8, 0x7,
    0xc0, 0xe, 0x0, 0x78, 0x3, 0xe0, 0x1d, 0xc0,
    0xe3, 0x3, 0x6, 0x18, 0x18,

    /* U+0079 "y" */
    0x18, 0x6, 0x38, 0x1c, 0x30, 0x30, 0x60, 0xc0,
    0xc3, 0x81, 0xc6, 0x1, 0x98, 0x3, 0x30, 0x6,
    0xc0, 0xf, 0x0, 0xe, 0x0, 0x18, 0x0, 0x60,
    0x1, 0xc0, 0x3f, 0x0, 0x3c, 0x0,

    /* U+007A "z" */
    0x3f, 0xf3, 0xfe, 0x0, 0xc0, 0x18, 0x3, 0x0,
    0x60, 0xc, 0x1, 0x80, 0x30, 0x6, 0x0, 0xff,
    0xcf, 0xfc,

    /* U+007B "{" */
    0x7, 0xf, 0xc, 0x1c, 0x18, 0x18, 0x18, 0x18,
    0x38, 0x30, 0xf0, 0xf0, 0x30, 0x30, 0x60, 0x60,
    0x60, 0x60, 0x60, 0x78, 0x30,

    /* U+007C "|" */
    0xc, 0x30, 0xc3, 0x8, 0x61, 0x86, 0x18, 0x63,
    0xc, 0x30, 0xc3, 0x18, 0x61, 0x86, 0x18, 0xc0,

    /* U+007D "}" */
    0xe, 0x7, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x1c,
    0xc, 0x6, 0x3, 0x1, 0xe0, 0xe0, 0xe0, 0x60,
    0x30, 0x18, 0x1c, 0xc, 0x6, 0xf, 0x7, 0x0,

    /* U+007E "~" */
    0x38, 0x2f, 0xcd, 0x1f, 0x61, 0xc0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 95, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 94, .box_w = 5, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 11, .adv_w = 138, .box_w = 7, .box_h = 6, .ofs_x = 2, .ofs_y = 10},
    {.bitmap_index = 17, .adv_w = 247, .box_w = 16, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 49, .adv_w = 219, .box_w = 14, .box_h = 20, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 84, .adv_w = 297, .box_w = 15, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 114, .adv_w = 241, .box_w = 13, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 140, .adv_w = 74, .box_w = 3, .box_h = 6, .ofs_x = 2, .ofs_y = 10},
    {.bitmap_index = 143, .adv_w = 119, .box_w = 7, .box_h = 21, .ofs_x = 2, .ofs_y = -4},
    {.bitmap_index = 162, .adv_w = 119, .box_w = 7, .box_h = 21, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 181, .adv_w = 141, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 8},
    {.bitmap_index = 192, .adv_w = 205, .box_w = 10, .box_h = 10, .ofs_x = 2, .ofs_y = 3},
    {.bitmap_index = 205, .adv_w = 80, .box_w = 4, .box_h = 7, .ofs_x = -1, .ofs_y = -3},
    {.bitmap_index = 209, .adv_w = 135, .box_w = 6, .box_h = 2, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 211, .adv_w = 80, .box_w = 3, .box_h = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 213, .adv_w = 124, .box_w = 13, .box_h = 21, .ofs_x = -2, .ofs_y = -2},
    {.bitmap_index = 248, .adv_w = 234, .box_w = 13, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 274, .adv_w = 130, .box_w = 6, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 286, .adv_w = 202, .box_w = 12, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 310, .adv_w = 201, .box_w = 14, .box_h = 16, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 338, .adv_w = 235, .box_w = 14, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 366, .adv_w = 202, .box_w = 13, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 392, .adv_w = 217, .box_w = 13, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 418, .adv_w = 210, .box_w = 12, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 442, .adv_w = 227, .box_w = 13, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 468, .adv_w = 217, .box_w = 12, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 492, .adv_w = 80, .box_w = 5, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 500, .adv_w = 80, .box_w = 6, .box_h = 16, .ofs_x = -1, .ofs_y = -4},
    {.bitmap_index = 512, .adv_w = 205, .box_w = 11, .box_h = 10, .ofs_x = 2, .ofs_y = 3},
    {.bitmap_index = 526, .adv_w = 205, .box_w = 11, .box_h = 7, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 536, .adv_w = 205, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 550, .adv_w = 202, .box_w = 10, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 570, .adv_w = 364, .box_w = 21, .box_h = 20, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 623, .adv_w = 258, .box_w = 16, .box_h = 16, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 655, .adv_w = 266, .box_w = 15, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 685, .adv_w = 254, .box_w = 15, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 715, .adv_w = 291, .box_w = 17, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 749, .adv_w = 236, .box_w = 14, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 777, .adv_w = 224, .box_w = 14, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 805, .adv_w = 272, .box_w = 16, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 837, .adv_w = 286, .box_w = 16, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 869, .adv_w = 109, .box_w = 5, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 879, .adv_w = 181, .box_w = 12, .box_h = 16, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 903, .adv_w = 253, .box_w = 16, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 935, .adv_w = 209, .box_w = 11, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 957, .adv_w = 336, .box_w = 20, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 997, .adv_w = 286, .box_w = 16, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1029, .adv_w = 296, .box_w = 17, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1063, .adv_w = 254, .box_w = 15, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1093, .adv_w = 296, .box_w = 17, .box_h = 19, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 1134, .adv_w = 256, .box_w = 15, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1164, .adv_w = 219, .box_w = 14, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1192, .adv_w = 207, .box_w = 13, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1218, .adv_w = 278, .box_w = 15, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1248, .adv_w = 251, .box_w = 16, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1280, .adv_w = 396, .box_w = 23, .box_h = 16, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 1326, .adv_w = 237, .box_w = 17, .box_h = 16, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1360, .adv_w = 228, .box_w = 14, .box_h = 16, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1388, .adv_w = 231, .box_w = 15, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1418, .adv_w = 118, .box_w = 9, .box_h = 21, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 1442, .adv_w = 124, .box_w = 5, .box_h = 21, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 1456, .adv_w = 117, .box_w = 9, .box_h = 21, .ofs_x = -2, .ofs_y = -4},
    {.bitmap_index = 1480, .adv_w = 205, .box_w = 10, .box_h = 10, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 1493, .adv_w = 176, .box_w = 11, .box_h = 2, .ofs_x = -1, .ofs_y = -1},
    {.bitmap_index = 1496, .adv_w = 211, .box_w = 5, .box_h = 3, .ofs_x = 5, .ofs_y = 14},
    {.bitmap_index = 1498, .adv_w = 240, .box_w = 13, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1518, .adv_w = 240, .box_w = 13, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1546, .adv_w = 201, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1564, .adv_w = 240, .box_w = 14, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1594, .adv_w = 215, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1612, .adv_w = 124, .box_w = 9, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1632, .adv_w = 243, .box_w = 14, .box_h = 16, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 1660, .adv_w = 240, .box_w = 12, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1686, .adv_w = 98, .box_w = 5, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1697, .adv_w = 100, .box_w = 10, .box_h = 21, .ofs_x = -4, .ofs_y = -4},
    {.bitmap_index = 1724, .adv_w = 213, .box_w = 13, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1752, .adv_w = 98, .box_w = 5, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1763, .adv_w = 372, .box_w = 21, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1795, .adv_w = 240, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1813, .adv_w = 224, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1831, .adv_w = 240, .box_w = 14, .box_h = 16, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 1859, .adv_w = 240, .box_w = 13, .box_h = 16, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1885, .adv_w = 144, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1897, .adv_w = 176, .box_w = 11, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1914, .adv_w = 146, .box_w = 8, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1929, .adv_w = 238, .box_w = 12, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1947, .adv_w = 197, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1965, .adv_w = 317, .box_w = 20, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1995, .adv_w = 194, .box_w = 14, .box_h = 12, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 2016, .adv_w = 197, .box_w = 15, .box_h = 16, .ofs_x = -2, .ofs_y = -4},
    {.bitmap_index = 2046, .adv_w = 183, .box_w = 12, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2064, .adv_w = 124, .box_w = 8, .box_h = 21, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 2085, .adv_w = 105, .box_w = 6, .box_h = 21, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 2101, .adv_w = 124, .box_w = 9, .box_h = 21, .ofs_x = -2, .ofs_y = -4},
    {.bitmap_index = 2125, .adv_w = 205, .box_w = 11, .box_h = 4, .ofs_x = 1, .ofs_y = 6}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 0, 1, 2, 0, 3, 4, 5,
    2, 6, 7, 8, 9, 10, 9, 10,
    11, 12, 0, 13, 14, 15, 16, 17,
    18, 19, 12, 20, 20, 0, 0, 0,
    21, 22, 23, 24, 25, 22, 26, 27,
    28, 29, 29, 30, 31, 32, 29, 29,
    22, 33, 34, 35, 3, 36, 30, 37,
    37, 38, 39, 40, 41, 42, 43, 0,
    44, 0, 45, 46, 47, 48, 49, 50,
    45, 51, 52, 52, 53, 48, 51, 51,
    46, 46, 54, 55, 56, 57, 45, 58,
    58, 59, 58, 60, 41, 0, 9, 9
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 0, 1, 2, 0, 3, 4, 5,
    2, 6, 7, 8, 9, 10, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 12,
    18, 19, 20, 21, 21, 0, 0, 0,
    22, 23, 24, 25, 23, 25, 25, 25,
    23, 25, 25, 26, 25, 25, 25, 25,
    23, 25, 23, 25, 3, 27, 28, 29,
    29, 30, 31, 32, 33, 34, 35, 0,
    36, 0, 37, 38, 37, 37, 37, 39,
    37, 38, 40, 41, 38, 38, 42, 42,
    37, 42, 37, 42, 43, 39, 44, 45,
    45, 46, 45, 47, 9, 0, 35, 9
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 4, 0, 0, 0,
    0, 2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1,
    16, 0, 10, -8, 0, 0, 0, 0,
    -19, -21, 2, 17, 8, 6, -14, 2,
    17, 1, 15, 4, 11, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    21, 3, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -11,
    0, 0, 0, 0, 0, -7, 6, 7,
    0, 0, -4, 0, -2, 4, 0, -4,
    0, -4, -2, -7, 0, 0, 0, 0,
    -4, 0, 0, -5, -5, 0, 0, -4,
    0, -7, 0, 0, 0, 0, 0, 0,
    0, 0, -4, -4, 0, 0, -10, 0,
    -43, 0, 0, -7, 0, 7, 11, 0,
    0, -7, 4, 4, 12, 7, -6, 7,
    0, 0, -20, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -13, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, -17, 0, -14,
    -2, 0, 0, 0, 0, 1, 14, 0,
    -11, -3, -1, 1, 0, -6, 0, 0,
    -2, -26, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -28, -3, 13,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 12, 0, 4,
    0, 0, -7, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 13, 3, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 2, 7, 4, 11, -4, 0, 0,
    7, -4, -12, -48, 2, 10, 7, 1,
    -5, 0, 13, 0, 11, 0, 11, 0,
    -33, 0, -4, 11, 0, 12, -4, 7,
    4, 0, 0, 1, -4, -6, 0, 11,
    28, 0, 28, 0, 0, 15, 5, 6,
    0, 0, 0, -13, 0, 0, 0, 0,
    1, -2, 0, 2, -6, -5, -7, 2,
    0, -4, 0, 0, 0, -14, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -23, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1,
    -19, 0, -22, 0, 0, 0, 0, -2,
    0, 35, -4, -5, 4, 4, -3, 0,
    -5, 4, 0, 0, -19, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -34, 0, 4, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 21,
    0, 0, -13, 0, 12, 0, -24, -34,
    -24, -7, 11, 0, 0, -24, 0, 4,
    -8, 0, -5, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 9,
    11, -43, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 2, 0,
    0, 0, 0, 0, 2, 2, -4, -7,
    0, -1, -1, -4, 0, 0, -2, 0,
    0, 0, -7, 0, -3, 0, -8, -7,
    0, -9, -12, -12, -7, 0, -7, 0,
    -7, 0, 0, 4, 0, -3, 0, 0,
    0, 2, -4, 0, 0, 0, 0, 4,
    -2, 0, 0, 0, -2, 4, 4, -1,
    0, 0, 0, -7, 0, -1, 0, 0,
    0, 0, 0, 1, 0, 5, -2, 0,
    -4, 0, -6, 0, 0, -2, 0, 11,
    -4, 0, 0, 0, 0, 0, 0, -1,
    1, -2, -2, 0, -4, 0, -4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, -2, 0, -4, -4, 0, 0, 0,
    0, 0, 1, 0, 0, -2, 0, -4,
    -4, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, -2,
    -5, 0, 0, -11, -2, -11, 7, 0,
    0, -7, 4, 7, 10, 0, -9, -1,
    -4, 0, -1, -17, 4, -2, 2, -19,
    4, 0, 0, 1, -18, 0, -19, -5,
    -31, -2, 0, -18, 0, 7, 5, 0,
    1, 0, 0, 0, 0, 0, -6, -5,
    0, 0, 0, 0, -4, 0, 0, 0,
    -4, 0, 0, 0, 0, 0, -2, -2,
    0, -2, -5, 0, 0, 0, 0, 0,
    0, 0, -4, -4, 0, -2, -4, -3,
    0, 0, -4, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -3, -3, 0,
    0, -2, 0, -7, 4, 0, 0, -4,
    2, 4, 4, 0, 0, 0, 0, 0,
    0, -2, 0, 0, 0, 0, 0, 2,
    0, 0, -4, 0, -4, -2, -4, 0,
    0, 0, 0, 0, 3, 0, 0, 0,
    -3, 0, 0, 0, -4, -5, 0, 0,
    11, -2, 1, -11, 0, 0, 10, -18,
    -18, -15, -7, 4, 0, -3, -23, -6,
    0, -6, 0, -7, 5, -6, -23, 0,
    -10, 0, 0, 2, -1, 3, -2, 0,
    4, 0, -11, -18, 0, -4, -8, -7,
    -8, -11, -10, -1, -7, -10, 0, 1,
    0, -4, 0, 0, 0, 2, 0, 4,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, -2, 0, -1,
    -4, 0, -6, -8, -8, -1, 0, -11,
    0, 0, 0, 0, 0, 0, -3, 0,
    0, 0, 1, -2, 0, 0, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 17,
    0, 0, 0, 0, 0, 0, 2, 0,
    0, 0, -4, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -6, 0,
    4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -2, 0, 0, 0,
    -7, 0, 0, 0, 0, -18, -11, 0,
    0, 0, -5, -18, 0, 0, -4, 4,
    0, -10, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, -7,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 4, 0, 2, -7, -7, 0, -4,
    -4, -4, 0, 0, 0, 0, 0, 0,
    -11, 0, -4, 0, -5, -4, 0, -8,
    -9, -11, -3, 0, -7, 0, -11, 0,
    0, 2, 0, 28, 0, 0, 0, 0,
    -5, 0, 0, -15, 0, 0, 0, 0,
    0, -33, -6, 12, 11, -3, -15, 0,
    4, -5, 0, -18, -2, -5, 4, -25,
    -4, 5, 0, 5, -12, -5, -13, -12,
    -15, 0, 0, -21, 0, 20, -2, 0,
    -2, 0, 0, 0, -2, -4, -10, -12,
    -1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, -2, -4, -5,
    0, 0, -7, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, -7, 0, 0, 7,
    -1, 5, 0, -8, 4, -2, -1, -9,
    -4, 0, -5, -4, -2, 0, -5, -6,
    0, 0, -3, -1, -2, -6, -4, 0,
    0, -4, 0, 4, -8, 0, 0, 0,
    0, 0, -7, -6, 0, -6, -6, 0,
    0, 0, 0, 0, 0, 0, 0, -7,
    4, 0, -5, 0, -2, -4, -11, -2,
    -2, -2, -1, -2, -4, -1, 0, 0,
    0, 0, 0, -4, -3, -3, 0, 0,
    0, 0, 4, -2, 0, -4, 0, 0,
    0, -2, -2, -3, -4, -3, 3, 14,
    -1, 0, -10, 0, -2, 7, 0, -4,
    -15, -5, 5, 0, 0, -17, -6, 4,
    -6, 2, 0, -2, -3, -11, 0, -5,
    2, 0, 0, -6, 0, 0, 0, 4,
    4, -7, -6, 0, 0, -4, -5, -4,
    -4, -6, 2, -7, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -6, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, -4, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, 0, -5, 0, 0,
    -4, -4, 0, 0, 0, 0, -4, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    -2, 0, 0, 0, -5, 0, -7, 0,
    0, 0, -12, 0, 2, -8, 7, 1,
    -2, -17, 0, 0, -8, -4, 0, -14,
    -9, -10, 0, 0, -15, -4, -14, -13,
    -17, 0, -9, 0, 3, 24, -8, 0,
    -10, -4, -1, -4, -6, -6, -13, -14,
    -8, 0, 0, -2, 0, 1, 0, 0,
    -25, -3, 11, 8, -8, -13, 0, 1,
    -11, 0, -18, -2, -4, 7, -32, -5,
    1, 0, 0, -23, -4, -18, -4, -26,
    0, 0, -25, 0, 21, -2, 0, -2,
    0, 0, 0, 0, -2, -13, -2, 0,
    0, 0, 0, 0, -11, 0, -3, 0,
    -1, -10, -17, 0, 0, -2, -5, -11,
    -4, 0, -2, 0, 0, 0, 0, -16,
    -4, -12, -11, -3, -6, -9, -4, -6,
    0, -7, -3, -12, -4, 0, 2, -7,
    -4, -7, 0, 0, -2, -12, 0, 0,
    -6, 0, 0, 0, 0, 4, 0, 2,
    -7, 14, 0, -4, -4, -4, 0, 0,
    0, 0, 0, 0, -11, 0, -4, 0,
    -5, -4, 0, -8, -9, -11, -3, 0,
    -7, 3, 14, 0, 0, 2, 0, 28,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0,
    0, 0, 0, 0, -2, -7, 0, 0,
    0, 0, 0, -2, 0, 0, 0, -4,
    -4, 0, 0, -7, -4, 0, 0, -7,
    0, 6, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 7, 3, -3,
    0, -11, -6, 0, 11, -12, -11, -7,
    -7, 14, 6, 4, -31, -2, 7, -4,
    0, -4, 4, -4, -12, 0, -4, 4,
    -5, -3, -11, -3, 0, 0, 11, 7,
    0, -19, 0, 1, -5, 10, -5, -13,
    -5, -12, -12, -4, 4, 0, -5, 0,
    -10, 0, 3, 12, -8, -13, -14, -9,
    11, 0, 1, -26, -3, 4, -6, -2,
    -8, 0, -8, -13, -5, -5, -3, 0,
    0, -8, -7, -4, 0, 11, 8, -4,
    -19, 0, -1, -5, 0, -12, -20, -11,
    -6, -12, -10, 0, 0, -5, 0, -7,
    -3, 0, -4, -6, 0, 6, -12, 4,
    0, 0, -19, 0, -4, -8, -6, -2,
    -11, -9, -12, -8, 0, -11, -4, -8,
    -7, -11, -4, 0, 0, 1, 17, -11,
    0, -8, -4, 0, -4, -7, -10, -10,
    -13, -5, 7, 0, -5, 0, -18, -4,
    2, 7, -11, -13, -7, -12, 12, -4,
    2, -33, -6, 7, -8, -6, -13, 0,
    -11, -15, -4, -4, -3, -4, -7, -11,
    -1, 0, 0, 11, 10, -2, -21, 0,
    -7, -8, 8, -13, -24, -12, -15, -18,
    -12, 0, 0, 0, 0, -4, 0, 0,
    4, -4, 7, 2, -7, 7, 0, 0,
    -11, -1, 0, -1, 0, 1, 1, -3,
    0, 0, 0, 0, 0, 0, -4, 0,
    0, 0, 0, 3, 11, -4, 0, -2,
    0, 0, 0, 0, -2, -4, 0, 0,
    1, 3, 0, 0, 0, 0, 3, 0,
    -3, 0, 13, 0, 6, 1, 1, -5,
    0, 7, 0, 0, 0, 3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 11, 0, 10, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -21, 0, -4, 6, 0, 11, 0, 0,
    35, 4, -7, -7, 4, 4, -2, 1,
    -18, 0, 0, 17, -21, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -24, 13, 49, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -6, 0, 0, -7, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -2, 0,
    -10, 0, 0, 1, 0, 0, 4, 45,
    -7, -3, 11, 10, -10, 4, 0, 0,
    4, 4, -5, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -46, 10,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -6, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 7, 0, 0, 0, 0, -30, -4,
    -3, -14, -17, 0, 0, -24, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, -10, 0,
    0, 0, -6, 4, -4, 0, 0, -10,
    -4, -8, 0, 0, -10, 0, -4, 0,
    -17, 0, -4, 0, 0, -29, -7, -14,
    -4, -13, 0, 0, -24, 0, -10, 0,
    0, 0, 0, 0, 0, 0, 0, -5,
    -6, -3, 0, 0, 0, 0, -8, 0,
    -8, 5, -4, 7, 0, -2, -8, -2,
    -6, -7, 0, -4, -2, -2, 2, -10,
    -1, 0, 0, 0, -31, -3, -5, 0,
    -8, 0, -2, -17, -3, 0, -3, -2,
    2, 0, 0, 0, 0, 0, -2, -6,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 5,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -8, 0, -2, 0, 0, 0, -7,
    4, 0, 0, 0, -10, -4, -7, 0,
    0, -10, 0, -4, 0, -17, 0, 0,
    0, 0, -34, 0, -7, -13, -18, 0,
    0, -24, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, -4, -5, -2, 1,
    0, 0, 6, -5, 0, 11, 17, -4,
    -4, -11, 4, 17, 6, 8, -10, 4,
    15, 4, 10, 8, 10, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    22, 17, -6, -3, 0, 0, 28, 15,
    28, 0, 0, 4, 0, 0, 0, 0,
    0, -10, 0, 0, 0, -10, 0, 0,
    0, 0, -8, -2, 0, 0, 0, -8,
    0, -4, 0, -17, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -24,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, -4, 0, 0, 0, 0, -6,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, 0, 0, 0, 0, 0,
    0, 0, 7, 0, 0, 0, 0, -30,
    -4, -3, -14, -17, 0, 0, -14, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, -8, 4, 0, -4,
    3, 6, 4, -11, 0, -1, -3, 4,
    0, 3, 0, 0, 0, 0, -9, 0,
    -3, -2, -7, 0, -3, -14, 0, 22,
    -8, 0, 0, -2, 0, -2, -6, -4,
    -10, -7, -4, 0, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    0, 0, 0, 0, 0, 0, 0, 0,
    7, 0, 0, 0, 0, -30, -4, -3,
    -14, -17, 0, 0, -24, 0, 0, 0,
    0, 0, 0, 18, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, -11, -4,
    -3, 11, -3, -4, -14, 1, -2, 1,
    -2, -10, 1, 8, 1, 3, 1, 3,
    -8, -14, -4, 0, -13, -7, -10, -15,
    -14, 0, -6, -7, -4, -5, -4, -2,
    5, -2, 0, -2, -1, 0, 5, -2,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -2, -4, -4,
    0, 0, -10, 0, -2, 0, -6, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -21, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -4, -4, 0,
    0, 0, 0, 0, -3, 0, 0, -6,
    -4, 4, 0, -6, -7, -2, 0, -10,
    -2, -8, -2, -4, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -24, 0, 11, -6, 0, -5, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, -2, 0, -8, 0, 0, 15, -5,
    -12, -11, 2, 4, 4, -1, -10, 2,
    5, 2, 11, 2, 12, -2, -10, 0,
    0, -14, 0, 0, -11, -10, 0, 0,
    -7, 0, -5, -5, 0, 5, 0, -5,
    0, -2, 0, -3, -11, -4, 0, 0,
    -3, 0, -7, 0, 0, 5, -8, 0,
    4, -4, 3, 0, 0, -12, 0, -2,
    -1, 0, -4, 4, -3, 0, 0, 0,
    -14, -4, -8, 0, -11, 0, 0, -17,
    0, 13, -6, 0, 0, 0, 2, 0,
    -4, -4, -11, 0, -4, 0, 0, 0,
    0, -2, 0, 0, 4, -5, 1, 0,
    0, -4, -2, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -22, 0,
    8, -3, 0, 1, 0, 0, 0, 0,
    0, -4, -4, 0
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 60,
    .right_class_cnt     = 47,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_classes,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t ui_font_Montserrat22 = {
#else
lv_font_t ui_font_Montserrat22 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 23,          /*The maximum line height required by the font*/
    .base_line = 4,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -2,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if UI_FONT_MONTSERRAT22*/

