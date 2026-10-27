"""The tower: which block is where, which blocks are legal to pull, and where a block goes next.

SHARED by every extraction method.  It knows nothing about the hand: no grippers, no stations.
Methods turn the target poses from here into hand poses themselves (see jenga/methods/).

Numbering: layer 0 is the bottom.  block_id = 3 * layer + slot at the start.  Slots 0 and 2 are the
edge blocks, slot 1 is the middle.  A block keeps its id for the whole game; when it is moved to the
top, its (layer, slot) changes.

Layer direction: even layers have their long axis along FIRST_LAYER_LONG_AXIS, odd layers the other
horizontal axis.  In a layer with long axis along y the three blocks sit side by side along x
(the "across" axis) and slot 0 is at the most negative x.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from jenga import geometry as geo


@dataclass
class BlockRecord:
    block_id: int
    layer: Optional[int]  # None for a block that is no longer part of the tower (dropped)
    slot: Optional[int]
    X_WB: np.ndarray  # 4x4, true pose in the world (updated from the simulator)
    loose: bool = False  # True if the block was dropped and lies somewhere it should not


class TowerState:
    def __init__(self, cfg, n_layers: Optional[int] = None):
        self.cfg = cfg
        self.dims = geo.block_dims(cfg)
        n = cfg.N_LAYERS_START if n_layers is None else n_layers
        self.blocks: Dict[int, BlockRecord] = {}
        self.slot_map: Dict[Tuple[int, int], int] = {}
        for layer in range(n):
            for slot in range(3):
                bid = 3 * layer + slot
                self.blocks[bid] = BlockRecord(bid, layer, slot, self.target_pose(layer, slot))
                self.slot_map[(layer, slot)] = bid

    # ------------------------------------------------------------ geometry of a slot
    def long_axis(self, layer: int) -> str:
        first = self.cfg.FIRST_LAYER_LONG_AXIS
        other = "x" if first == "y" else "y"
        return first if layer % 2 == 0 else other

    def across_axis(self, layer: int) -> np.ndarray:
        """Unit vector (world) along which the three blocks of this layer are side by side."""
        return np.array([1.0, 0.0]) if self.long_axis(layer) == "y" else np.array([0.0, 1.0])

    def target_pose(self, layer: int, slot: int) -> np.ndarray:
        """4x4 world pose of the block body frame (bottom-face centre) at (layer, slot)."""
        L, W, H = self.dims
        gap = self.cfg.BLOCK_GAP_M
        cx, cy = self.cfg.TOWER_CENTER_XY
        across = (slot - 1) * (W + gap)
        z = self.cfg.PLATFORM_HEIGHT_M + layer * (H + gap)
        if self.long_axis(layer) == "y":  # body x along world y: yaw 90 deg, blocks side by side along x
            return geo.make_pose(cx + across, cy, z, np.pi / 2)
        return geo.make_pose(cx, cy + across, z, 0.0)

    # ------------------------------------------------------------ reading the state
    def block_pose(self, block_id: int) -> np.ndarray:
        return self.blocks[block_id].X_WB

    def update_poses(self, poses: Dict[int, np.ndarray]) -> None:
        """Copy the true poses from the simulator into the tower state."""
        for bid, X in poses.items():
            self.blocks[bid].X_WB = np.array(X, dtype=float)

    def layer_blocks(self, layer: int) -> List[int]:
        return [self.slot_map[(layer, s)] for s in range(3) if (layer, s) in self.slot_map]

    def top_layer(self) -> int:
        return max(layer for layer, _ in self.slot_map)

    def is_complete(self, layer: int) -> bool:
        return len(self.layer_blocks(layer)) == 3

    def highest_complete_layer(self) -> Optional[int]:
        full = [layer for layer in range(self.top_layer() + 1) if self.is_complete(layer)]
        return max(full) if full else None

    # ------------------------------------------------------------ legal pulls (the TOWER rule)
    def pull_candidates(self) -> List[int]:
        """Blocks that are legal to pull: every block except those in the highest complete layer
        and in any incomplete layer above it (the layer being built).
        A method applies its own physical-reach rule on top of this (e.g. end-pinch: edge blocks only)."""
        top_full = self.highest_complete_layer()
        out = []
        for (layer, _slot), bid in sorted(self.slot_map.items()):
            if top_full is not None and layer >= top_full:
                continue
            if self.blocks[bid].loose:
                continue
            out.append(bid)
        return out

    # ------------------------------------------------------------ where a pulled block goes
    def placement_slots(self) -> List[Tuple[int, int]]:
        """Empty slots in the layer being built, edges first and the middle last.
        If the top layer is complete this is the new layer above it."""
        top = self.top_layer()
        layer = top + 1 if self.is_complete(top) else top
        return [(layer, s) for s in (0, 2, 1) if (layer, s) not in self.slot_map]

    # ------------------------------------------------------------ changing the state
    def remove_from_slot(self, block_id: int) -> None:
        rec = self.blocks[block_id]
        if rec.layer is not None:
            self.slot_map.pop((rec.layer, rec.slot), None)
        rec.layer, rec.slot = None, None

    def place(self, block_id: int, layer: int, slot: int) -> None:
        """Record that block_id now sits at (layer, slot) (its pose comes from update_poses)."""
        self.remove_from_slot(block_id)
        rec = self.blocks[block_id]
        rec.layer, rec.slot, rec.loose = layer, slot, False
        self.slot_map[(layer, slot)] = block_id

    def mark_loose(self, block_id: int) -> None:
        """The block was dropped: it leaves the tower, stays wherever it fell, and is never picked again."""
        self.remove_from_slot(block_id)
        self.blocks[block_id].loose = True
