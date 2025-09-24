import time 
import carla

framerate = 60
dt = 1/framerate

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(120)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = dt
    _ = world.apply_settings(settings)
    while True:
        _ = world.tick()
        tick_start = time.time()
        # state_machine.step(dt)
        compute_time = time.time() - tick_start
        if compute_time < dt:
            time.sleep(dt - compute_time)

if __name__ == '__main__':
    main()
