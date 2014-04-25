#!/usr/bin/ruby

Indent = "    "
Width = 16

filename = ARGV[0]
basename = File.basename(filename, File.extname(filename))

print <<END
const char #{basename}[] = {
END

def put_chara(chara)
  print ", " unless @first
  @first = false
  if @col == Width
    print "\n"
    @col = 0
  end
  print Indent if @col == 0
  print "%#04x" % chara
  @col += 1
end

@first = true
@col = 0

open(filename) do | f |
  while line = f.gets do
    line.each_byte do |chara|
      put_chara(chara)
    end
  end
  put_chara(0)
  print " };\n"
end
