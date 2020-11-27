file ./target/thumbv7m-none-eabi/debug/blinky
target extended-remote 3333
monitor reset halt
load

set auto-load safe-path /

define stop
    monitor reset halt
end

define run
    continue
end

define q
    quit
end