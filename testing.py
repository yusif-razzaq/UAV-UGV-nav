import cv2
import numpy as np

def positionToPixels(position):
    # Given parameters
    fov_horizontal_deg = 90  # Horizontal FOV in degrees
    fov_vertical_deg = 90  # Vertical FOV in degrees
    image_width = 1000  # Image width in pixels
    image_height = 1000  # Image height in pixels

    # Coordinates of the images
    coord_image1 = np.array(position)  # (x, y, z) for image 1

    # Convert FOV from degrees to radians
    fov_horizontal_rad = np.radians(fov_horizontal_deg)
    fov_vertical_rad = np.radians(fov_vertical_deg)

    # Calculate focal length in horizontal and vertical directions
    focal_length_horizontal = image_width / (2 * np.tan(fov_horizontal_rad / 2))
    focal_length_vertical = image_height / (2 * np.tan(fov_vertical_rad / 2))

    # Convert world coordinates to pixel offsets
    def world_to_pixel(coord):
        pixel_x = (focal_length_horizontal * coord[0]) / coord[2] + (image_width / 2)
        pixel_y = (focal_length_vertical * coord[1]) / coord[2] + (image_height / 2)
        return int(pixel_x), int(pixel_y)

    # Convert coordinates to pixel offsets
    pixel_offset = world_to_pixel(coord_image1)
    print("Pixel offset:", pixel_offset)
    return pixel_offset


# Read the images
image1 = cv2.imread('imgs/img_0_0.png')
image2 = cv2.imread('imgs/img_0_1.png')

# Calculate the width and height of the first image
height1, width1 = image1.shape[:2]

# Create a blank canvas to place the images side by side
result = 255 * np.ones((max(image1.shape[0], image2.shape[0]), width1 + image2.shape[1], 3), dtype=np.uint8)

# Place the first image on the canvas
result[:height1, :width1, :] = image1

# Place the second image to the right of the first image
result[:image2.shape[0], width1:, :] = image2

# Display the result
cv2.imshow('Overlayed Images', result)
# cv2.waitKey(0)
cv2.destroyAllWindows()

positionToPixels([0, 0, 10])
