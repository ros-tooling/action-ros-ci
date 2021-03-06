module.exports = {
	"*": [
		"eslint --cache --fix --ignore-path .eslintignore",
		"prettier --write --ignore-path .eslintignore --ignore-unknown",
		// Note: doing the build here ensures we omit unstaged changes
		() => "npm run build",
		() => "git add dist/index.js",
		// same here. Only test staged changes.
		() => "npm run test",
	],
};
