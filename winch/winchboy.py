import math

LINE_THICKNESS = 0.035  # inches
LINE_LENGTH_INCHES = 150 * 12  # total line length to store, in inches
FIXED_LENGTH = 0.6 # inches, times 3 spools equals 5.4 inches, gives us .6 margin on 6 inch spool
INNER_RADIUS = 0.295 # 6mm to in
PACKING_EFFICIENCY = 0.95  

"""
RESULTS:
 length: 1.8in
 inner radius: 0.500 in
 outer radius: 0.675 in
 edge radius = OR + some margin = 0.7in
"""
def calc_outer_radius(inner_radius, length, packing_efficiency=1.0, tolerance=0.0001):

    def simulate(inner_r, outer_r, length, efficiency):
        total_length = 0
        current_radius = inner_r
        while current_radius <= outer_r + tolerance:
            wraps = math.floor(length / LINE_THICKNESS)
            circumference = 2 * math.pi * current_radius
            layer_length = wraps * circumference * efficiency  # account for packing efficiency
            total_length += layer_length
            current_radius += LINE_THICKNESS
        return total_length

    r_low = inner_radius
    r_high = inner_radius + 10  # Arbitrary
    result_radius = r_high

    while r_high - r_low > tolerance:
        r_mid = (r_low + r_high) / 2
        stored_length = simulate(inner_radius, r_mid, length, packing_efficiency)

        if stored_length >= LINE_LENGTH_INCHES:
            result_radius = r_mid
            r_high = r_mid  # Try smaller outer radius
        else:
            r_low = r_mid  # Need more space

    return result_radius

outer_radius = calc_outer_radius(
    INNER_RADIUS, FIXED_LENGTH, PACKING_EFFICIENCY
)
print(f"inner radius: {INNER_RADIUS:.3f} in")
print(f"outer radius: {outer_radius:.3f} in")
print(f"volume: {math.pi * (outer_radius**2 - INNER_RADIUS**2) * FIXED_LENGTH:.3f} in^3")
