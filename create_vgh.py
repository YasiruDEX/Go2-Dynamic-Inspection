import struct

def write_vgh(filename):
    # 4 Nodes for 10x10 polygon centered at origin
    nodes = [
        {"id": 0, "pos": (5.0, 5.0, 0.0)},
        {"id": 1, "pos": (5.0, -5.0, 0.0)},
        {"id": 2, "pos": (-5.0, -5.0, 0.0)},
        {"id": 3, "pos": (-5.0, 5.0, 0.0)},
    ]
    
    # Polygon edges (0<->1, 1<->2, 2<->3, 3<->0)
    poly_edges = {
        0: [1, 3],
        1: [0, 2],
        2: [1, 3],
        3: [2, 0]
    }
    
    with open(filename, "wb") as f:
        # graph_size
        f.write(struct.pack("=Q", len(nodes)))
        
        for n in nodes:
            nid = n["id"]
            x, y, z = n["pos"]
            
            # id
            f.write(struct.pack("=Q", nid))
            
            # position
            f.write(struct.pack("=fff", x, y, z))
            
            # bools and int
            # is_covered, is_frontier, is_navpoint, is_boundary
            # free_direct
            f.write(struct.pack("=????i", False, False, False, True, 0))
            
            # connect_size + array
            f.write(struct.pack("=Q", 0))
            
            # poly_size + array
            polys = poly_edges.get(nid, [])
            f.write(struct.pack("=Q", len(polys)))
            for p in polys:
                f.write(struct.pack("=Q", p))
                
            # contour_size + array
            f.write(struct.pack("=Q", 0))

write_vgh("saved_vgraphs/10x10_polygon.vgh")
print("Saved saved_vgraphs/10x10_polygon.vgh")
