#version 330 core

// 这定义了一个输入变量ourColor，它是一个三维向量（vec3），通常用于表示RGB颜色。
// 这个变量从顶点着色器传递过来，包含了每个顶点或片段的颜色信息
in vec3 ourColor;
out vec4 FragColor;   // RGBA（红绿蓝和透明度）

// 这是片段着色器的主函数，每个片段都会执行这个函数
void main()
{
    FragColor = vec4(ourColor, 1.0f);     // 设置透明度分量为1.0（即完全不透明）
}
