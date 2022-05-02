#version 330 core
out vec4 fragColor;

// Additional information for lighting
in vec4 normal_worldSpace;
in vec4 position_worldSpace;

uniform int wire = 0;
uniform float red = 1.0;
uniform float green = 1.0;
uniform float blue = 1.0;
uniform float alpha = 1.0;

void main() {

    vec4 lightPos = vec4(-2.0, 2.0, -3.0 , 1.0);
    vec3 lightColor = vec3(1.0f, alpha, 0.0f);
    vec4 lightDir = normalize(-lightPos + position_worldSpace);
    float c = clamp(dot(-normal_worldSpace, lightDir), 0, 1);

    float a = 1;
    if(wire == 1) {
        a = 0;
    }

    fragColor = vec4(wire*red + a*red * c * lightColor[0], wire*green + a*green * c * lightColor[0], wire*blue + a*blue * c * lightColor[0], alpha);
}
