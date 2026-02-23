#version 330 core
out vec4 FragColor;

in vec3 FragPos;  
in vec3 Normal;  

uniform vec3 lightDir; 
uniform vec3 viewPos;
uniform vec3 objectColor;

void main(){
    // Ambient (The dark shadows in the folds)
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * objectColor;

    // Diffuse (The direct light hitting the fabric)
    // Use gl_FrontFacing to ensure the normal always points towards the camera
    // This fixes the dark creases and specular artifacts on the back side of the cloth!
    vec3 norm = normalize(Normal);
    if (!gl_FrontFacing) {
        norm = -norm;
    }

    // Diffuse
    vec3 lightDirNorm = normalize(-lightDir); 
    float diff = abs(dot(norm, lightDirNorm)); 
    vec3 diffuse = 0.8 * diff * objectColor;

    // Specular 
    float specularStrength = 0.1;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDirNorm, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 8);
    vec3 specular = specularStrength * spec * vec3(1.0); // White highlight

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}