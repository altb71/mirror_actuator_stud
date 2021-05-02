class IIR_filter{
     public:
     
        IIR_filter(float T, float Ts);
        IIR_filter(float T, float Ts, float K);
        IIR_filter(float w0, float D, float Ts, float K);
        IIR_filter(float *b, float *a, int nb_, int na_);
                    
        float operator()(float u){
            return filter((double)u);
         }
        virtual     ~IIR_filter();
        void        reset(float);
        float       filter(double);
    
    private:

        unsigned int nb;
        unsigned int na;
        double *B;
        double *A;
        double *uk;
        double *yk;
        double K;
};
