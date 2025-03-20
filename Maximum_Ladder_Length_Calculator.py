def max_ladder_length(h, f, w):
    """
    Calculate the maximum ladder length that can pass around a right-angled hallway corner.
    :param h: Width of the vertical hallway
    :param f: Width of the horizontal hallway
    :return: Maximum ladder length
    """
    buh = (h-w)**(2/3) + (f-w)**(2/3)
    guh = buh**(3/2)
    return guh

# Example usage
if __name__ == "__main__":
    h = float(input("Enter the width of the vertical hallway: "))
    f = float(input("Enter the width of the horizontal hallway: "))
    w = float(input("Enter the width of the beam: "))
    ladder_length = max_ladder_length(h, f, w)
    print(f"The maximum ladder length that can be carried around the corner is: {ladder_length:.2f}")