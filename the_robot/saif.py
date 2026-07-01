import numpy as np
import cv2
import time



def take_picture_and_binarize():
    """Captures a single frame, saves copies for debugging, and returns a matrix."""
    
    # 1. INITIALIZE THE WEBCAM
    camera = cv2.VideoCapture(0)
    time.sleep(1) # Let the lens adjust to the lighting
    
    # 2. SNAP THE PHOTO
    ret, frame = camera.read()
    camera.release()
    
    if not ret:
        print("Error: Could not connect to the webcam.")
        return None

    # --- SAVE THE RAW PHOTO ---
    # This saves the full-color picture so you can check for dark desk borders or bad shadows.
    cv2.imwrite('debug_raw_photo.jpg', frame)
    print("Raw photo saved as debug_raw_photo.jpg")

    # 3. CONVERT TO GRAYSCALE
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 4. COMPRESS TO THE 50x50 GRID
    small_grid = cv2.resize(gray_image, (50, 50), interpolation=cv2.INTER_NEAREST)
    
    # 5. OTSU'S BINARIZATION
    _, thresholded_img = cv2.threshold(small_grid, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # --- SAVE THE THRESHOLDED PHOTO ---
    # This saves a picture of the pure black and white grid so you can see exactly what the AI sees.
    cv2.imwrite('debug_binarized_grid.jpg', thresholded_img)
    print("Binarized grid saved as debug_binarized_grid.jpg")
    
    # 6. CONVERT TO 1s AND 0s
    binary_matrix = thresholded_img // 255
    final_matrix = 1 - binary_matrix
    
    return final_matrix

def greedy_set_cover(binarized_matrix, footprint_size=3):
    """
    Calculates the minimum stops to cover all infection in a binarized matrix.
    Assumes footprint_size is an odd number (e.g., 3x3, 5x5) representing
    the area the UV light covers at a single stop.
    """
    matrix = np.array(binarized_matrix, dtype=int)
    rows, cols = matrix.shape
    offset = footprint_size // 2
    final_stops = []
    
    while np.sum(matrix) > 0:
        best_stop = None
        max_dirt_covered = 0
        
        # 1. EVALUATE EVERY POSSIBLE STOPPING POSITION
        for r in range(rows):
            for c in range(cols):
                covered_dirt = 0
                for fr in range(r - offset, r + offset + 1):
                    for fc in range(c - offset, c + offset + 1):
                        if 0 <= fr < rows and 0 <= fc < cols:
                            if matrix[fr, fc] == 1:
                                covered_dirt += 1
                                
                # 2. THE GREEDY CHOICE
                if covered_dirt > max_dirt_covered:
                    max_dirt_covered = covered_dirt
                    best_stop = (r, c)
                    
        if max_dirt_covered == 0:
            break
            
        final_stops.append(best_stop)
        
        # 3. ERASE THE COVERED DIRT
        center_r, center_c = best_stop
        for fr in range(center_r - offset, center_r + offset + 1):
            for fc in range(center_c - offset, center_c + offset + 1):
                if 0 <= fr < rows and 0 <= fc < cols:
                    matrix[fr, fc] = 0
                    
    return final_stops


def main():

    print("Snapping a live photo from the webcam...")

    # This reaches into the other file, clicks the webcam, and grabs the matrix
    live_matrix = take_picture_and_binarize()

    # Feed it directly into the Greedy Algorithm
    calculated_path = greedy_set_cover(live_matrix, footprint_size=20)

    print(f"Total stops required: {len(calculated_path)}\n")
    for step, coordinate in enumerate(calculated_path):
        print(f"Stop {step + 1}: Move robot to matrix coordinate {coordinate} and blast UV.")


# THE MAIN ROBOT EXECUTION LOOP
if __name__ == "__main__":
    main()