function payload = Create_Payload(w, l, h, m)
    
    payload = Box(in2m(w), in2m(l), in2m(h), lb2kg(m), true);

end