file ./target/thumbv7m-none-eabi/debug/car-with-two-wheels
target remote :3333
monitor reset halt
load

define stop
    monitor reset halt
end

define run
    continue
end

define q
    monitor reset halt
    quit
end