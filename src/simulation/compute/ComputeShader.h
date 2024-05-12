#ifndef COMPUTE_SHADER_H
#define COMPUTE_SHADER_H

class ComputeShader {
public:
    ComputeShader() {}
    ComputeShader(const char * programText);
    ComputeShader(const ComputeShader &other) = delete;
    ComputeShader &operator=(const ComputeShader &other) = delete;
    ComputeShader(ComputeShader &&other): initialized(other.initialized), id(other.id) {}
    ComputeShader &operator=(ComputeShader &&other);
    ~ComputeShader();

    /** @brief Enable the compute shader as part of the current state **/
    void enable();

    /**
     * @brief Dispatch the compute shader with given group sizes using glComputeDispatch()
     * @param xGroups 
     * @param yGroups 
     * @param zGroups 
     */
    void dispatch(unsigned int xGroups, unsigned int yGroups, unsigned int zGroups);

    /** @brief Disable the compute shader, call after you no longer need it to avoid global state pollution **/
    void disable();

    /**
     * @brief Get opengl shader id
     * @return unsigned int 
     */
    unsigned int getId() const { return id; }

private:
    bool initialized = false;
    unsigned int id = 0;
};

#endif
