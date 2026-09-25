public class CRT {
    private int[] multiples;
    private int[] mods;
    private int lcm;
    public CRT(int[] multi, int[] remainders) {
        multiples = multi;
        mods = remainders;
    }
    
    public int euclids_algorithm(int a, int b) {
        
        if (b > a) {
            a ^= b;
            b = a^b;
            a = a^b;
        }
        
        if (b <= 0) {
            return a;
        }
        
        return euclids_algorithm(b, a % b);
    }
    
    public int gcd() {
        int curr_gcd = multiples[0];
        for (int i = 1; i < multiples.length; i++) {
            curr_gcd = euclids_algorithm(curr_gcd, multiples[i]); 
        }
        
        return curr_gcd;
    }
    
    public void lcm() {
        int product = 1;
        for (int i : multiples) {
            product *= i;
        }
        
        lcm = product/gcd();
    }
    
    public int mod_inverse(int M, int k) {
        int i = 1;
        while ((M*i) % k != 1) {
            i++;
        }
        
        return i;   
    }
    
    public int solve() {
        int answer = 0;
        lcm();
        for (int i = 0; i < multiples.length; i++) {
            int M = lcm/multiples[i];
            int inverse = mod_inverse(M, multiples[i]);
            answer += (M*inverse*mods[i]);
        }
        
        if (answer < 0) {
            answer += lcm;
        }
        
        return answer % lcm;
    }
    
    public int get_lcm() {
        return lcm;
    }
}

