#ifndef BYTEORDERING_H
#define BYTEORDERING_H

#define HTOL16(val) (val)
#define HTOL32(val) (val)
#define LTOH16(val) (val)
#define LTOH32(val) (val)

#define htol16(h) (h)
#define htol32(h) (h)
#define ltoh16(l) (l)
#define ltoh32(l) (l)

uint16_t read16(const uint8_t *p);
uint32_t read32(const uint8_t *p);

#endif
