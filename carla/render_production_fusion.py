"""Render four native cameras and measured forward LiDAR without inference.

Inputs are frozen numerical results. This module neither changes predictions nor
patches the original front-camera renderer; it is safe to call alongside it.
"""
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fuse_camera_lidar import PALETTE, font, project


class Labels:
    """Place compact labels inside one view with deterministic overlap avoidance."""

    def __init__(self, draw, bounds, object_boxes=()):
        self.draw, self.bounds, self.occupied = draw, bounds, []
        self.object_boxes = [tuple(map(float, box)) for box in object_boxes]
        self.placed = []

    def __call__(self, xy, text, color, size=20, anchor_box=None):
        face = font(size)
        box = self.draw.textbbox((0, 0), text, font=face)
        width, height = box[2]-box[0]+10, box[3]-box[1]+8
        left, top, right, bottom = self.bounds
        anchor_x, anchor_y = map(float, xy)
        step = height+5
        positions = [(anchor_x+dx, anchor_y+dy)
                     for dy in [0, -step, step, -2*step, 2*step, -3*step, 3*step,
                                -4*step, 4*step, -5*step, 5*step]
                     for dx in [0, -width/2, width/2]]
        # Search empty space across the view as a fallback for dense traffic.
        # Merely moving past an existing label must not cover a small vehicle.
        positions.extend((x, y) for y in np.arange(top+3, bottom-height-2, step)
                         for x in (left+3, anchor_x, (left+right-width)/2, right-width-3))

        def overlap(rect, other, padding):
            return (max(0, min(rect[2]+padding, other[2])-max(rect[0]-padding, other[0])) *
                    max(0, min(rect[3]+padding, other[3])-max(rect[1]-padding, other[1])))

        candidates = []
        for proposed_x, proposed_y in positions:
            x = float(np.clip(proposed_x, left+3, max(left+3, right-width-3)))
            y = float(np.clip(proposed_y, top+3, max(top+3, bottom-height-3)))
            rect = (x, y, x+width, y+height)
            object_overlap = sum(overlap(rect, box, 5) for box in self.object_boxes)
            label_overlap = sum(overlap(rect, box, 4) for box in self.occupied)
            candidates.append((object_overlap, label_overlap, abs(x-anchor_x)+abs(y-anchor_y), rect))
        _, _, _, rect = min(candidates, key=lambda item: item[:3])
        x, y = rect[:2]
        if anchor_box is not None:
            bx0, by0, bx1, by1 = map(float, anchor_box)
            bx = float(np.clip(x+width/2, bx0, bx1))
            by = float(np.clip(y+height/2, by0, by1))
            lx = float(np.clip(bx, x, x+width))
            ly = float(np.clip(by, y, y+height))
            self.draw.line((bx, by, lx, ly), fill=color, width=1)
        elif abs(x-anchor_x)+abs(y-anchor_y) > 10:
            self.draw.line((anchor_x, anchor_y+height/2, x+width/2, y+height/2), fill=color, width=1)
        self.occupied.append(rect)
        self.placed.append((rect, (x+5-box[0], y+4-box[1]), text, face, color))
        # A later target's leader may pass behind an earlier label. Repaint the
        # compact label layer so connectors never obscure existing text.
        for placed_rect, text_xy, placed_text, placed_face, placed_color in self.placed:
            self.draw.rectangle(placed_rect, fill=(9, 16, 24))
            self.draw.text(text_xy, placed_text, font=placed_face, fill=placed_color)
        return rect


def color_for(item):
    # Identity colors remain stable across cameras even when LiDAR is unavailable.
    # Only an observed association may add points or a distance to its label.
    return PALETTE[(item['display_track_id']-1) % len(PALETTE)]


def render_views(images, xyz, ground, projections, detections):
    """Return four cameras and 3D/BEV images from frozen observations.

    All cameras must be RGB uint8 1280x720 arrays. ``projections`` maps camera
    names to (sampled point indices, pixel coordinates). Detections use the
    shared display_track_id supplied by the caller, never simulator actor IDs.
    Camera-only observations draw no LiDAR points, boxes, range or guessed depth.
    The LiDAR plot is made only from supplied measured returns; the forward-only
    profile cannot acquire a rear target merely because its image was detected.
    """
    xyz = np.asarray(xyz)
    ground = np.asarray(ground, dtype=bool)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or ground.shape != (len(xyz),):
        raise ValueError('Expected Nx3 LiDAR points and one ground flag per point')
    if not np.isfinite(xyz).all():
        raise ValueError('LiDAR display points must be finite')
    cameras = ('front', 'rear', 'side_left', 'side_right')
    if any(item.get('camera') not in cameras for item in detections):
        raise ValueError('Every detection must identify one of the four native cameras')
    colors = np.full((len(xyz), 3), [111, 132, 151], np.uint8)
    colors[ground] = [48, 99, 84]
    results = {}
    point_owners = {}
    for name in cameras:
        rgb = np.asarray(images[name])
        if rgb.shape != (720, 1280, 3) or rgb.dtype != np.uint8:
            raise ValueError('Expected native 1280x720 uint8 RGB camera images')
        picture = Image.fromarray(rgb.copy())
        draw = ImageDraw.Draw(picture)
        indices, uv = projections[name]
        lookup = {int(index): pixel for index, pixel in zip(indices, uv)}
        items = [item for item in detections if item['camera'] == name]
        labels = Labels(draw, (0, 0, 1280, 720), [item['bbox_xyxy'] for item in items])
        # Draw all boxes first so later boxes do not cross already placed labels.
        for item in items:
            ids = np.asarray(item['sampled_point_indices'], dtype=int)
            if item['fusion_status'] != 'associated' and len(ids):
                raise ValueError('Camera-only target cannot have measured LiDAR point membership')
            if np.any(ids < 0) or np.any(ids >= len(xyz)):
                raise ValueError('Association refers to an invalid sampled point')
            color = color_for(item)
            for index in ids:
                if int(index) not in lookup:
                    raise ValueError('Associated point lies outside its camera projection')
                if int(index) in point_owners and point_owners[int(index)] != item['display_track_id']:
                    raise ValueError('One measured point cannot be colored as two different targets')
                point_owners[int(index)] = item['display_track_id']
                x, y = lookup[int(index)]
                radius = 4 if name.startswith('side_') else 1
                draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=color)
            colors[ids] = color
            draw.rectangle(tuple(item['bbox_xyxy']), outline=color, width=8 if name.startswith('side_') else 2)
        for item in items:
            x0, y0, _, _ = item['bbox_xyxy']
            size = 48 if name.startswith('side_') else 22
            text = f"#{item['display_track_id']} {item['class_name']} {item['score']:.2f}"
            if item['fusion_status'] == 'associated':
                text += f"  {item['range_m']:.1f}m"
            labels((x0, y0-size-10), text, color_for(item), size, anchor_box=item['bbox_xyxy'])
        results['camera' if name == 'front' else name] = picture

    # Center the observer on the measured forward sector, without adding returns.
    position, target = np.array([-40., -75., 85.]), np.array([25., 0., -2.])
    forward = target-position
    forward /= np.linalg.norm(forward)
    right = np.cross([0, 0, 1.], forward)
    right /= np.linalg.norm(right)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((forward, right, np.cross(forward, right)))
    transform[:3, 3] = position
    intrinsic = np.array([[520., 0, 320], [0, 520., 330], [0, 0, 1.]])
    indices, pixels, depth = project(xyz, np.eye(4), transform, intrinsic, 640, 660)
    lidar = Image.new('RGB', (640, 1080), (8, 15, 23))
    cloud = ImageDraw.Draw(lidar)
    for index in np.argsort(depth)[::-1]:
        x, y = pixels[index]
        cloud.ellipse((x-1, y-1, x+1, y+1), fill=tuple(map(int, colors[indices[index]])))
    cloud.rectangle((0, 660, 639, 1079), fill=(11, 21, 30))
    scale, center_x, center_y = 4.4, 320., 1050.

    def bev(points):
        return np.column_stack((center_x+points[:, 1]*scale, center_y-points[:, 0]*scale))

    # The range grid is only an orientation guide, not sensor returns.
    grid = Image.new('RGB', (640, 420), (11, 21, 30))
    grid_draw = ImageDraw.Draw(grid)
    grid_origin_y = center_y-660
    for distance in (10, 20, 40, 60, 80):
        radius = distance*scale
        grid_draw.ellipse((center_x-radius, grid_origin_y-radius, center_x+radius, grid_origin_y+radius),
                          outline=(30, 46, 59), width=1)
    grid_draw.line((center_x, 0, center_x, 419), fill=(30, 46, 59))
    grid_draw.line((0, grid_origin_y, 639, grid_origin_y), fill=(30, 46, 59))
    lidar.paste(grid, (0, 660))
    cloud = ImageDraw.Draw(lidar)
    small = bev(xyz)
    valid = ((small[:, 0] >= 0) & (small[:, 0] < 640) &
             (small[:, 1] >= 664) & (small[:, 1] < 1080))
    for index in np.flatnonzero(valid):
        x, y = small[index]
        cloud.point((x, y), fill=tuple(map(int, colors[index])))
    # The arrow marks the sensor origin and positive x without editorial text.
    cloud.polygon(((320, 1042), (314, 1056), (320, 1053), (326, 1056)), fill=(205, 215, 224))
    labels_3d = Labels(cloud, (0, 0, 640, 658))
    labels_bev = Labels(cloud, (0, 662, 640, 1080))
    # Merge repeated cross-camera observations of one target for LiDAR display.
    targets = {}
    for item in detections:
        if item['fusion_status'] != 'associated':
            continue
        target = targets.setdefault(item['display_track_id'], {'ids': set(), 'item': item})
        target['ids'].update(item['sampled_point_indices'])
    for identifier, target in targets.items():
        item = target['item']
        ids = np.asarray(sorted(target['ids']), dtype=int)
        if not len(ids):
            raise ValueError('Associated target requires measured LiDAR points')
        points, color = xyz[ids], color_for(item)
        small = bev(points)
        x0, y0 = small.min(axis=0)
        x1, y1 = small.max(axis=0)
        if x1 >= 0 and x0 < 640 and y1 >= 664 and y0 < 1080:
            cloud.rectangle((max(0, x0-2), max(664, y0-2), min(639, x1+2), min(1079, y1+2)),
                            outline=color, width=2)
            distance = float(np.median(np.linalg.norm(points, axis=1)))
            labels_bev((x0, y0-24), f'#{identifier} {distance:.1f}m', color, 18)
        keep, pos, _ = project(np.median(points, axis=0)[None, :], np.eye(4), transform,
                               intrinsic, 640, 660)
        if len(keep):
            labels_3d((pos[0, 0], pos[0, 1]-24), f'#{identifier}', color, 18)
    results['lidar'] = lidar
    return results
