patterns = [
    ('Vehicle Cross', 'Park'),
    ('Vehicle Cross', 'Retrograde'),
    ('Retrograde', 'Brake'),
    ('Vehicle Cross', 'Turn Around'),
    ('Follow Vehicle', 'Brake'),
    ('Follow Vehicle', 'Cut In'),
    ('Follow Vehicle', 'Change Lane'),
    ('Follow Lane', 'Change Lane'),
    ('Pedestrian Cross', 'Park'),
    ('Pedestrian Cross', 'Vehicle Cross'),
    ('Pedestrian Walk', 'Turn Around')]


def get_behavior_pattern(index):
    return patterns[index]


def get_all_behaviors():
    return patterns
