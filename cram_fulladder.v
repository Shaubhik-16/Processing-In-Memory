module cram_full_adder(input a, input b, input cin, output s, output cout); //in memory addition
wire p,g; // propagation and generation
p= a^b;
g= a&b;
s= p^cin;
cout= g|(p&cin);
endmodule
