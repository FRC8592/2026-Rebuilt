package frc.robot.subsystems;

public class CRT {
    private int[] multiples;
    private int[] mods;
    private int lcm;
    public CRT(int[] multi, int[] remainders) {
        multiples = multi;
        mods = remainders;
    }
    
    public int euclidsAlgorithm(int a, int b) {
        
        if (b <= 0) {
            return a;
        }
        
        return euclidsAlgorithm(b, a % b);
    }
    
    public int gcd(int a, int b) {
        return euclidsAlgorithm(a, b); 
    
    }
    
    public void lcm() {
        lcm = 1;
        for (int i : multiples) {
            lcm = lcm*i/gcd(lcm,i);
        }
    }
    
    public int extendedEuclideanAlgorithm(int M, int k, int T1, int T2) {
        
        if (k == 0) { 
            return T1; 
        }
        
        int Q = M / k;
        int R = M % k;
        
        int nextT = T1 - Q * T2;
        
        return extendedEuclideanAlgorithm(k, R, T2, nextT);
    }
    
    public int modInverse(int M, int k) {
        int val = extendedEuclideanAlgorithm(M, k, 1,0);  
        
        return (val % k + k) % k;
    }
    
    public int solve() {
        int answer = 0;
        lcm();
        for (int i = 0; i < multiples.length; i++) {
            int M = lcm/multiples[i];
            int inverse = modInverse(M, multiples[i]);
            answer += (M*inverse*mods[i]);
        }
        
        if (answer < 0) {
            answer += lcm;
        }
        
        return answer % lcm;
    }
    
    public int getLcm() {
        return lcm;
    }
}


