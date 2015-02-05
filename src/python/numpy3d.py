
import math
import numpy

def normalized(vector):
    return vector / numpy.linalg.norm(vector)

def angleaxis(theta, axis):
    axis = numpy.array(axis)
    axis = axis / math.sqrt(numpy.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return numpy.array(
        [[ aa + bb - cc - dd, 2.0 * (bc + ad),   2.0 * (bd - ac)  ],
         [ 2.0 * (bc - ad),   aa + cc - bb - dd, 2.0 * (cd + ab)  ],
         [ 2.0 * (bd + ac),   2.0 * (cd - ab),   aa + dd - bb - cc]])

def angleaxis44(theta, axis):
    axis = numpy.array(axis)
    axis = axis / math.sqrt(numpy.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return numpy.array(
        [[ aa + bb - cc -dd, 2.0 * (bc + ad),   2.0 * (bd - ac),   0.0 ],
         [ 2.0 * (bc - ad),  aa + cc - bb - dd, 2.0 * (cd + ab),   0.0 ],
         [ 2.0 * (bd + ac),  2.0 * (cd - ab),   aa + dd - bb - cc, 0.0 ],
         [ 0.0,              0.0,               0.0,               1.0 ]])

def rpy2rot(r, p, y):
    cr = math.cos(r);
    sr = math.sin(r);
    cp = math.cos(p);
    sp = math.sin(p);
    cy = math.cos(y);
    sy = math.sin(y);
    return numpy.array(
        [[ cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy ],
         [ cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy ],
         [ -sp  , sr*cp           , cr*cp            ]])

def rpy2rot44(r, p, y):
    cr = math.cos(r);
    sr = math.sin(r);
    cp = math.cos(p);
    sp = math.sin(p);
    cy = math.cos(y);
    sy = math.sin(y);
    return numpy.array(
        [[ cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy, 0.0 ],
         [ cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy, 0.0 ],
         [ -sp  , sr*cp           , cr*cp,            0.0 ],
         [ 0.0,   0.0,              0.0,              1.0 ]])

def rot2rpy(R):

    if abs(R[0,0]) < abs(R[2,0]) and abs(R[1,0]) < abs(R[2,0]):
        # cos(p) is nearly = 0
        sp = -R[2,0]
        if sp < -1.0:
            sp = -1.0
        elif sp > 1.0:
            sp = 1.0

        pitch = math.asin(sp) # -pi/2< p < pi/2
            
        roll = math.atan2(sp * R[0,1] + R[1,2],  # -cp*cp*sr*cy
                          sp * R[0,2] - R[1,1])  # -cp*cp*cr*cy
            
        if R[0,0] > 0.0: # cy > 0
            if roll < 0.0:
                roll += PI
            else:
                roll -= PI

        sr = math.sin(roll);
        cr = math.cos(roll);
        if sp > 0.0:
            yaw = math.atan2(sr * R[1,1] + cr * R[1,2], # sy*sp
                             sr * R[0,1] + cr * R[0,2]) # cy*sp
        else:
            yaw = math.atan2(-sr * R[1,1] - cr * R[1,2],
                             -sr * R[0,1] - cr * R[0,2])
    else:
        yaw = math.atan2(R[1,0], R[0,0]);
        sa = math.sin(yaw);
        ca = math.cos(yaw);
        pitch = math.atan2(-R[2,0], ca * R[0,0] + sa * R[1,0])
        roll = math.atan2(sa * R[0,2] - ca * R[1,2], -sa * R[0,1] + ca * R[1,1])

    return numpy.array([roll, pitch, yaw])
