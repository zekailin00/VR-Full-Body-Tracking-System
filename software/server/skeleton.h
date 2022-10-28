struct vec3
{
    float x;
    float y;
    float z;
};

struct Skeleton
{
    vec3 LeftUpperLeg;
    vec3 LeftLowerLeg;
    vec3 LeftFoot;

    vec3 RightUpperLeg;
    vec3 RightLowerLeg;
    vec3 RightFoot;

    vec3 LeftUpperArm;
    vec3 LeftLowerArm;
    vec3 LeftHand;

    vec3 RightUpperArm;
    vec3 RightLowerArm;
    vec3 RightHand;

    vec3 waist;
    vec3 chest;
    vec3 head;

    vec3 avatarPosition;
};