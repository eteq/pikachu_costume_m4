import time  #stdlib

import board
import neopixel

NPIX_STRIP = 64
MAX_BRIGHT = .2

builtin_npx = neopixel.NeoPixel(board.NEOPIXEL, 1)
builtin_npx.brightness = MAX_BRIGHT
builtin_npx[0] = (255,0,0)

npx_strip = neopixel.NeoPixel(board.D4, NPIX_STRIP, brightness=MAX_BRIGHT)

i = 0
while True:
    if i % 2 == 0:
        npx_strip.fill((255,255,0))
        builtin_npx.fill((255,255,0))
        print('yellow')
    else:
        npx_strip.fill((255,255,255))
        builtin_npx.fill((255,255,255))
        print('white')
    builtin_npx.show()
    npx_strip.show()

    time.sleep(1)

    i += 1