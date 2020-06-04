
/***************************************************
 * 
 * Fitting of a Polynomial using Cramer Rule
 * 
 * Hague Nusseck @ electricidea
 * 
 * --> see curve_fit.cpp for more details on how it 
 * works and how to use it.
 * 
 * Distributed as-is; no warranty is given.
 * 
 * ************************************************/

// class definition
class curve_fit {
    public:
        curve_fit(uint8_t degree=2);
        bool init(uint8_t degree);
        void learn(double x, double y);
        double predict(double x);
        double predict(double x, double outside_value);
        void get_coefficients(double values[]);
        String get_formula(uint8_t decimals = 6);
        uint32_t get_order();
        void reset();
        double max_x() const { return max_x_; }
        double min_x() const { return min_x_; }
        int count() const { return N; }
        double estimate_max_y(uint32_t steps = 100);
        double estimate_min_y(uint32_t steps = 100);
        int tag;
        String name;
    private:
        // Mindex generates the index for adressing the matrices
        uint32_t Mindex(uint32_t i, uint32_t j);
        double determinant(double *mainmatrix);
        int order = -1;
        double *M;
        double *b;
        // y = a[n]*x^n + ... + a[1]*x + a[0]
        double *a;
        // Number of learned x, y pairs
        uint32_t N = 0;
        double max_x_, min_x_;
};