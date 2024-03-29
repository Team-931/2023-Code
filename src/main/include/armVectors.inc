constexpr double lowPost[] = {21, 46, 111}, 
                 lowPostBack[] = {-95, 300, 37},
                 lowCubeBack[] = {-81, 286, 56},

                 highPost[] = {72, 48, 130},
                 highCubeBack[] = {-93, 304, -11.5},

                 openInFront[] = {35, 30, 224},
                 openInBack[] = {-63, 125, 224},
                 foldedDown[] = {-19, 125, 224},

                 conePickup1[] = {57, 25, 130},
                 conePickup2[] = {57, 26, 85},
                 cubePickup1[] = {50, 49, 108},
                 cubePickup2[] = {50, 50, 109},
                 coneOnFloor[] = {-11.5, 121, 97.5},
                 coneOnFloorBack1[] = {-35, 282, 55},
                 coneOnFloorBack2[] = {-32, 275, 57},
                 cubeOnFloorBack[] = {-50, 244, 43},
                 cubeOnFloor[] = {-11.5, 120, 95};

using Constants::Arm::ht, Constants::Arm::len1, Constants::Arm::len2, Constants::Arm::len3;
using Constants::pi;
constexpr const double (&makeAngles(double fwdIn, double htIn, double ang3, 
                        bool reverseElbow = false))[3] {
        htIn -= ht;
        double cos2 = ((len1*len1 + len2*len2) - (htIn*htIn + fwdIn*fwdIn))/(2*len1*len2),
               theta2 = acos(cos2),
               sin2 = sin(theta2);
        theta2 /= (2*pi);
        if (reverseElbow) {
               sin2 = - sin2;
               theta2 += .5; 
        };
        double h0 = cos2 - len1/len2, 
                s1 = atan2(h0*htIn+sin2*fwdIn,sin2*htIn-h0*fwdIn) / (2*pi);
        return {360*s1, 360*(theta2 - s1), ang3};
}

double const foo[] = {makeAngles(len1,ht-len2,0)[0], makeAngles(len1,ht-len2,0)[1]};