file ./target/thumbv7m-none-eabi/debug/blinky
target remote :3333
monitor reset halt
load

set auto-load safe-path /

define reset
    monitor reset halt
end

define start
    continue
end

define q
    quit
end