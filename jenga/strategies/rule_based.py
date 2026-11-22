"""RULE-BASED SELECTION  (team code: Katie)

Chooses a block using the exact tower state (course approach (b)): for example favour outside blocks
in upper layers, or blocks whose neighbours give the most support.  Runs on equal-friction towers
(and also on varied-friction towers, as a comparison for the force strategy).
"""
import numpy as np

# ---- BEGIN CODE TO BE WRITTEN ----
def layer_stable(tower, candidate)->bool:
    #checks if you pulled this block could the layer be stable afterwards based on physics
    #for layer to be stable afterwards it needs to be either the center block left
    #or any combination of two blocks left
    layer = tower.blocks[candidate].layer
    blocksleftIDs = tower.layer_blocks(layer)
    slotsleft = {tower.blocks[b].slot for b in blocksleftIDs}
    slotsleft.discard(tower.blocks[candidate].slot) #what would be left after pulling it
    #need both sides or middle and one side
    if len(slotsleft) ==2: #any two blocks left is fine/stable
        return True
    elif 1 in slotsleft:#if only one block left has to be middle
        return True
    else:
        return False
    
def threeblocksleft(tower, layer)->bool:
    #returns true if the layer has three blocks left
    if layer is None:
        return False
    blocksleftIDs = tower.layer_blocks(layer)
    if len(blocksleftIDs) == 3:
        return True
    else:
        return False
    
def blockdisturbance(tower, block, cfg)->float:
    blockinfo = tower.blocks[block]
    loc_now = tower.block_pose(block)
    loc_ideal = tower.target_pose(blockinfo.layer,blockinfo.slot)
    distmoved = np.linalg.norm(loc_now[:3, 3] - loc_ideal[:3, 3])
    flips = [np.diag(d) for d in ([1, 1, 1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1])]
    angmoved = min(np.arccos(np.clip((np.trace((loc_ideal[:3, :3] @ F).T @ loc_now[:3, :3]) - 1) / 2, -1.0, 1.0))
                   for F in flips)
    return float(distmoved / cfg.RB_DISTURB_M + angmoved / np.radians(cfg.RB_DISTURB_DEG))

def layerdisturbance(tower, layer, cfg):
    if layer is None:
        return 0
    blocks = tower.layer_blocks(layer)
    values = [blockdisturbance(tower, block, cfg) for block in blocks]
    return float(np.mean(values)) if values else 0.0


def blocksbelow(tower,candidatelayer,slot):
    count=0
    checked=0
    for layer in range(candidatelayer%2,candidatelayer,2):
        checked+=1
        if (layer, slot) in tower.slot_map:
            count+=1
    if checked == 0:
        return 1.0
    else:
        return count/checked
    
def column_intact(tower, block) -> bool:
    """True if no block is missing from this block's column: the same slot in every same-direction layer below it
    (blocksbelow == 1).  Pulling a block whose column is hollow ended the game every time in V2's end-pinch
    games (93 of 93, DECISIONS O-8)."""
    info = tower.blocks[block]
    return blocksbelow(tower, info.layer, info.slot) >= 1.0

def towerimbalance(tower,block,cfg):
    info = tower.blocks[block]
    if info.slot == 1:
        return 0.5
    own = blocksbelow(tower,info.layer,info.slot)
    other = blocksbelow(tower,info.layer,2-info.slot)
    return 0.5+0.5*(own-other)

def end_weights(cfg) -> dict:
    """The layer weights for edge blocks: the RB_W_* settings (V2, D-23)."""
    return dict(HEIGHT=cfg.RB_W_HEIGHT, FULL_LAYERS_NEAR=cfg.RB_W_FULL_LAYERS_NEAR, SELFSTILL=cfg.RB_W_SELFSTILL,
                OTHERNEARSTILL=cfg.RB_W_OTHERNEARSTILL, SELF_FULL=cfg.RB_W_SELF_FULL,
                CLEAR_OF_TOP=cfg.RB_W_CLEAR_OF_TOP, CLEAR_OF_BOTTOM=cfg.RB_W_CLEAR_OF_BOTTOM)

def layerscore(tower, layers,cfg, w=None):
    # each factor weighted 0 to 1 and then weighted using config weights
    # w: the layer weights (end_weights(cfg) for edge blocks, cfg.RB_MID_W for middles)
    w = end_weights(cfg) if w is None else w
    scores = {}
    toplayer = max(tower.top_layer(), 1)
    for candidatelayer in layers:
        layer_above = min((l for l in range(toplayer+1) if l > candidatelayer), default=None)
        layer_below = max((l for l in range(toplayer+1) if l < candidatelayer), default=None)
        scores[candidatelayer] = 0
        # each score scaled from 0 to 1, using sigmoid for distances
        height = 1-candidatelayer/toplayer #it gets more points for being lower
        fulllayerself = 1.0*threeblocksleft(tower,candidatelayer) #layer still has all three blocks
        topfull = tower.highest_complete_layer()
        cleartop = 0.0 if topfull is not None and candidatelayer >= topfull - cfg.RB_CLEAR_OF_TOP_LAYERS else 1.0 #not the layer(s) the top rests on
        clearbottom = 0.0 if candidatelayer == 0 else 1.0 #not the bottom layer (layer 1 is not affected)
        fulllayersnear = 0.5*(threeblocksleft(tower,layer_above)+threeblocksleft(tower,layer_below))
        selfstill = 1/(1+layerdisturbance(tower,candidatelayer,cfg))
        othernearstill = 1/(1+layerdisturbance(tower,layer_above,cfg)+layerdisturbance(tower,layer_below,cfg))
        scores[candidatelayer] = (w["HEIGHT"]    * height
                                + w["FULL_LAYERS_NEAR"] * fulllayersnear
                                + w["SELFSTILL"]     * selfstill
                                + w["OTHERNEARSTILL"]      * othernearstill
                                + w["SELF_FULL"]     * fulllayerself
                                + w["CLEAR_OF_TOP"]  * cleartop
                                + w["CLEAR_OF_BOTTOM"] * clearbottom)
    return scores

def blockscore(tower, candidates, cfg):
    scores = {}
    for candidate in candidates:
        blockinfo = tower.blocks[candidate]
        edge = 1.0 if blockinfo.slot in (0, 2) else 0.0
        selfstill = 1/(1+blockdisturbance(tower,candidate,cfg))
        blockfracbelow = blocksbelow(tower,blockinfo.layer,blockinfo.slot)
        balanced = towerimbalance(tower,candidate,cfg)
        scores[candidate] = (cfg.RB_W_EDGE      * edge
                           + cfg.RB_W_BALANCED  * balanced
                           + cfg.RB_W_SELFSTILL_BLOCK * selfstill
                           + cfg.RB_W_BLOCKS_BELOW* blockfracbelow)
    return scores

def select_block(tower, candidates, cfg, rng, sensors=None) -> int:
    """Choose a block using the exact tower state.

    Inputs and output: see jenga/strategies/__init__.py.
    You may use everything in `tower`: tower.blocks (poses, layer, slot), tower.layer_blocks(layer),
    tower.top_layer().  Not allowed: per-block friction, contact forces.
    """
    safecands = [candidate for candidate in candidates if layer_stable(tower,candidate)]
    if not safecands:
        safecands = list(candidates)
    ends = [c for c in safecands if tower.blocks[c].slot != 1]
    middles = [c for c in safecands if tower.blocks[c].slot == 1]
    goodends = ends
    if cfg.RB_COLUMN_FILTER:  # edge blocks only: leave out hollow-column ones while an intact one is left (O-8)
        goodends = [c for c in ends if column_intact(tower, c)]
    # Which kind: the extraction method sets what is offered (end-pinch: edges, push: middles, mixed: both).
    # With both on offer, a coin picks edge with probability RB_P_END; with no good edge (none, or only hollow
    # ones) a middle is taken instead, and with no middle an edge.  No coin is drawn otherwise, so end-only
    # and middle-only games use the random stream exactly as before.
    if goodends and middles:
        kind = "end" if rng.random() < cfg.RB_P_END else "middle"
    else:
        kind = "middle" if middles else "end"
    if kind == "middle":
        safecands, w = middles, cfg.RB_MID_W
    else:
        safecands, w = (goodends or ends), end_weights(cfg)
    layers = sorted({tower.blocks[candidate].layer for candidate in safecands})
    #layers_3bl = [layer for layer in layers if threeblocksleft(tower,layer)]
    #pool = layers_3bl if layers_3bl else layers
    layscore = layerscore(tower,layers,cfg,w)

    best = max(layscore.values())
    toplayers = [l for l, s in layscore.items() if s >= best - cfg.RB_LAYER_TIE]
    chosenlayer = int(rng.choice(toplayers))

    candpool = [c for c in safecands if tower.blocks[c].layer == chosenlayer]
    candscore = blockscore(tower,candpool,cfg)
    weights = np.array([candscore[c] for c in candpool], dtype=float) + 1e-9
    p = weights / weights.sum()
    return int(rng.choice(candpool, p=p))
    # totscore = {block: layscore[tower.blocks[block].layer]+candscore[block] for block in safecands}
    # cands_3bl = [candidate for candidate in candidates if threeblocksleft(tower, tower.blocks[candidate].layer)]
    # candpool = cands_3bl if cands_3bl else safecands
# ---- END CODE TO BE WRITTEN ----
    


    


    
