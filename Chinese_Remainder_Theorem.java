class CRT {
    private int[] multiples;
    private int[] mods;
    private int lcm;
    public CRT(int[] multi, int[] remainders) {
        multiples = multi;
        mods = remainders;
    }
    
    public int[] prime_factorize(int num) {
        int[] test_primes = new int[] {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47};
        int[] factors = new int[test_primes.length];
        for (int i = 0; i < test_primes.length; i++) {
            factors[i] = 0;
        }
        
        for (int i = 0; i < test_primes.length && num >= test_primes[i]; i++) {
            while (num % test_primes[i] == 0) {
                factors[i]++;
                num /= test_primes[i];
            }
        }
        
        return factors;
    }
    
    public void lcm() {
        int[] test_primes = new int[] {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47};
        int[][] factor_list = new int[multiples.length][test_primes.length];
        for (int i = 0; i < multiples.length; i++) {
            factor_list[i] = prime_factorize(multiples[i]);
        }
        
        int[] finished_list = new int[test_primes.length];
        for (int i = 0; i < test_primes.length; i++) {
            int max = factor_list[0][i];
            for (int j = 1; j < multiples.length; j++) {
                if (factor_list[j][i] > max) {
                    max = factor_list[j][i];
                }
            }
            
            finished_list[i] = max;
        }
        
        lcm = 1;
        for (int i = 0; i < finished_list.length; i++) {
            lcm *= Math.pow(test_primes[i], finished_list[i]);
        }
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

public class Main
{
	public static void main(String[] args) {
	    int[] multiples = {2,3,5};
	    int[] mods = {1,2,3};
		CRT crt = new CRT(multiples, mods);
		System.out.println(crt.solve() + " + " + crt.get_lcm() + "k");
	}
}
