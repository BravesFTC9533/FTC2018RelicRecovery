package org.firstinspires.ftc.teamcode;

/**
 * Created by dmill on 2/17/2018.
 */

public class Quad<A,B,C,D> {
    private final A a;
    private final B b;
    private final C c;
    private final D d;

    public Quad(A a, B b, C c, D d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public A getA() { return  a;}
    public B getB() { return  b;}
    public C getC() { return  c;}
    public D getD() { return  d;}

    @Override
    public int hashCode()
    {
        int result = 17;
        result = 31 * result + a.hashCode();
        result = 31 * result + b.hashCode();
        result = 31 * result + c.hashCode();
        result = 31 * result + d.hashCode();

        return result;

    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Quad)) return false;
        Quad quado = (Quad) o;
        return this.a.equals(quado.getA()) &&
                this.b.equals(quado.getB()) &&
                this.c.equals(quado.getC()) &&
                this.d.equals(quado.getD());

    }
}