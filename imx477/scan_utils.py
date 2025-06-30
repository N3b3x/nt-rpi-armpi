def generate_polar_scan_angles(start=-180, stop=180, step=60):
    """
    Generate a list of angles for a polar scan pattern.
    Args:
        start (int): Starting angle in degrees (default -180)
        stop (int): Ending angle in degrees (default 180)
        step (int): Step size in degrees (default 60)
    Returns:
        List[int]: List of angles covering the specified range.
    """
    return list(range(start, stop + 1, step)) 