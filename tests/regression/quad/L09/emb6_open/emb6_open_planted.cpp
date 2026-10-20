// Negative control of invariant L-5a's post-link check (tools/ci/check_emb6_open.py): an image whose symbols name the
// measurement build, so the check must find emb6_open in it. The marker is an object the program reads (a volatile read is never
// removed), so neither the compiler nor the linker's section garbage collection drops it.
namespace marv::planted {

volatile int emb6_open_trigger = 0;

}  // namespace marv::planted

int main() { return marv::planted::emb6_open_trigger; }
