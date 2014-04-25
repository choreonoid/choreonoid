#!/usr/bin/ruby

require 'jcode'
$KCODE = "UTF8"

Indent = "    "
Width = 80

filename = ARGV[0]
basename = File.basename(filename, File.extname(filename))

print <<END
const char* #{basename} =
END

# \

open(filename) do | f |
  while line = f.gets do
    line.gsub!(/\\/, '\\\\\\')
    line.gsub!(/"/, '\"')
    line.gsub!(/\n/, '\\\n')
    text = line.split(//u)
    pos = 0
    while pos < text.size
      col = 0
      print Indent + "\""
      while pos < text.size and col < Width
        print text[pos]
        col += (text[pos].size > 1) ? 2 : 1
        pos += 1
      end
      print "\""
      print "\n" if pos < text.size
    end
    print "\n" unless f.eof
  end
  print ";\n"
end
