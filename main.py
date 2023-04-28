import pygame
import gcs.control_map
import utils.flight_conditions

SETTINGS_FILE = "C:\\Users\\pa.perrier\\Desktop\\PREDATOR\\parameters\\settings.json"

def manual():
    # Initialize Pygame
    pygame.init()

    # Initialize the Xbox controller
    controller = pygame.joystick.Joystick(0)
    controller.init()

    # Check flight conditions
    utils.flight_conditions.main()

    has_taken_off = False
    while not has_taken_off:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                has_taken_off = True

            # Check for 'Start' button press
            if event.type == pygame.JOYBUTTONDOWN and event.button == 7:
                has_taken_off = True

    # Main game loop
    en_route = True
    while en_route:
        # Handle events
        for event in pygame.event.get():
            if not event.type == pygame.JOYBUTTONDOWN and event.button == 7:
                gcs.control_map.handle_event(event)
            else:
                is_landing = True

    # Quit Pygame
    pygame.quit()


if __name__ == "__main__":
    manual()