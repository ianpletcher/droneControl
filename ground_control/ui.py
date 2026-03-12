import pygame

from data_cmd import send_command_to_air_pi



def select_target_by_click(click_pos, tracking_data, app_state):
    """Finds a target at the click position and sends command to Air Pi"""
   
    found_target = False
    new_target_id = None
   
    for data in tracking_data:
        # Setting starting and ending coordinates of bounding box from tracking data
        (start_x, start_y, end_x, end_y) = data['bbox']

        # If the click position is within the bounding box, we have found a target
        if start_x < click_pos[0] < end_x and start_y < click_pos[1] < end_y:
            new_target_id = data['id']
            found_target = True
            break
           
    with app_state.target_lock:
        app_state.current_target_id = new_target_id
       
    if found_target:
        print(f"Clicked new target: ID {new_target_id}")
    else:
        print("Clicked empty space, clearing target.")
       
    # Not really sending directly to Air Pi, maybe change name?
    send_command_to_air_pi({'target_id': new_target_id})
    
# -----------------------------------------------------------------------------------------------
# Pygame UI Functions
# -----------------------------------------------------------------------------------------------
def handle_input(events, ui_state, app_state, tracking_data):
    """Handle pygame events"""
    for event in events:
        if event.type == pygame.QUIT:
            ui_state['running'] = False
           
        elif event.type == pygame.MOUSEBUTTONDOWN:
            click_pos = pygame.mouse.get_pos()
            # Handle click in main thread to avoid race conditions
            select_target_by_click(click_pos, tracking_data, app_state)
               
        elif event.type == pygame.KEYDOWN:
            # Clear target
            if event.key == pygame.K_c:
                print("Target cleared by 'C' key")
                with app_state.target_lock:
                    app_state.current_target_id = None
                send_command_to_air_pi({'target_id': None})
                   
            # Quit
            elif event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                ui_state['running'] = False


def render_graphics(screen, frame, tracking_data, current_target_id, ui_state, fonts):
    """Render frame with tracking overlays"""
   
    if frame is None:
        # Display "Waiting for video" message
        screen.fill((30, 30, 30)) # Dark grey background
        wait_text = "Waiting for video stream... (Radxa USB connected?)"
        text_surface = fonts['main'].render(wait_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=screen.get_rect().center)
        screen.blit(text_surface, text_rect)
        pygame.display.flip()
        return

    # Convert numpy array to pygame surface
    # The frame is already RGB, but GStreamer gives it to us rotated.
    # We must rotate it and flip it to be upright.
    frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
   
    screen.blit(frame_surface, (0, 0))
   
    frame_width, frame_height = screen.get_size()
   
    # Draw tracked objects
    for data in tracking_data:
        (start_x, start_y, end_x, end_y) = data['bbox']
       
        is_target = (data['id'] == current_target_id)
       
        # Red for target, green for others
        color = (255, 0, 0) if is_target else (0, 255, 0)
        thickness = 3 if is_target else 2
       
        # Draw bounding box
        pygame.draw.rect(
            screen, color,
            (start_x, start_y, end_x - start_x, end_y - start_y),
            thickness
        )
       
        # Draw label
        label_text = f"ID:{data['id']} {data['label']}"
        if is_target:
            label_text = f"★ {label_text} ★"
       
        text_surface = fonts['small'].render(label_text, True, color, (0,0,0))
        screen.blit(text_surface, (start_x, start_y - 25))
   
    # Draw instructions
    instructions = [
        "Click object to track | C: Clear target | Q/ESC: Quit"
    ]
    y_offset = frame_height - 30
    for instruction in instructions:
        instr_surface = fonts['small'].render(instruction, True, (200, 200, 200), (0, 0, 0))
        screen.blit(instr_surface, (10, y_offset))
        y_offset += 25
   
    pygame.display.flip()