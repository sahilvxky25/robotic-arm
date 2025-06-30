import requests

# -- CONFIGURATION --
# Replace with your ESP32's IP address
ESP32_IP_ADDRESS = "192.168.1.100" 

def send_coordinates(x, y, z):
    """
    Sends target coordinates to the ESP32 robotic arm.

    Args:
        x (float): The target X coordinate.
        y (float): The target Y coordinate.
        z (float): The target Z coordinate.
    """
    url = f"http://{ESP32_IP_ADDRESS}/set"
    params = {'x': x, 'y': y, 'z': z}
    
    try:
        response = requests.get(url, params=params, timeout=5)
        response.raise_for_status()  # Raises an exception for bad status codes (4xx or 5xx)
        
        print("Successfully sent coordinates.")
        print(f"Response from server: {response.text}")

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    print("Robotic Arm PC Controller")
    print("Enter the target coordinates for object placement.")
    
    try:
        target_x = float(input("Enter Target X (mm): "))
        target_y = float(input("Enter Target Y (mm): "))
        target_z = float(input("Enter Target Z (mm): "))
        
        send_coordinates(target_x, target_y, target_z)

    except ValueError:
        print("Invalid input. Please enter numeric values.")